/**
* @file      object_tracker.cpp
* @brief     Per-target core of the camera/LiDAR sensor fusion object
             tracker. Logic extracted from object_tracker_node.cpp so that
             one instance can be replicated per tracked target class. This is
             the "Generic" tracker type; role-specific behavior lives in the
             derived ToiTracker and FormationVehicleTracker classes.
*/

#include <avt_341_nav/perception/tracking/object_tracker.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

#include <avt_341_nav/core/coord_transform.hpp>
#include <avt_341_nav/core/eigen_dto_conversion.hpp>
#include <avt_341_nav/core/string_utils.hpp>
#include <avt_341_nav/node/node_utils.h>

namespace avt_341_nav {
namespace perception {

ObjectTracker::ObjectTracker(
    rclcpp::Node* node, const std::string& target_class,
    const ObjectTrackerSettings& params,
    const core::CoordTransformer& coord_transformer,
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr target_contacts_publisher,
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr leader_odom_publisher)
    : node_(node),
      logger_(node->get_logger().get_child(core::SanitizeIdentifier(target_class))),
      target_class_(target_class),
      target_ns_(core::SanitizeIdentifier(target_class)),
      params_(params),
      frame_ids_(params.frames, node::GetLeadingNodeNamespace(node->get_namespace())),
      coord_transformer_(coord_transformer),
      target_contacts_publisher_(std::move(target_contacts_publisher)), 
	  leader_odom_publisher_(std::move(leader_odom_publisher)
{
    // Initialize the IMM filter (CV + CTR + NM).
    filter_ = std::make_shared<avt_341_nav::perception::filtering::IMMFilter>(
        1.0 / params_.filter.estimator_rate,
        params_.filter.process_variance,
        params_.filter.measurement_variance,
        params_.filter.imm_cv_init_prob,
        params_.filter.imm_ctr_init_prob,
        params_.filter.imm_nm_init_prob,
        params_.filter.imm_persistence_prob);
    filter_->SetInitialPosition(Eigen::Vector3d::Zero());

    const rclcpp::Time now = node_->get_clock()->now();
    last_valid_detection_time_ = now;
    last_valid_detection_callback_time_ = now;
    last_valid_target_time_ = now;
    last_lidar_seen_time_ = now;

    CreatePerTargetPublishers();
    // Configure the SAC segmentation filter.
    sac_segmentation_.setOptimizeCoefficients(true);
    sac_segmentation_.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    sac_segmentation_.setEpsAngle((M_PI / 180.0) * 5);
    sac_segmentation_.setAxis(Eigen::Vector3f{ 0.0, 1.0, 0.0 });
    sac_segmentation_.setMethodType(pcl::SAC_RANSAC);
    sac_segmentation_.setMaxIterations(1000);
    sac_segmentation_.setDistanceThreshold(0.3);
    sac_segmentation_.setNumberOfThreads(0);
}

void ObjectTracker::CreatePerTargetPublishers() {
    detection_publisher_ =
        node_->create_publisher<vision_msgs::msg::Detection3D>(
            target_ns_ + "/detection_3d", 1);

    if (params_.publish.odometry) {
        odometry_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(
            target_ns_ + "/odometry/raw", 1);
    }

    tracked_target_odometry_publisher_ =
        node_->create_publisher<nav_msgs::msg::Odometry>(
            "avt_341/odometry/estimated/" + target_ns_, 1);
}

void ObjectTracker::TrackingTick(const TrackerSensorContext& context) {
    camera_info_ = context.camera_info;

    // LOST is sticky against the state machine below; only a fresh camera
    // detection of the target ends it and resumes normal tracking.
    if (state_ == TrackerState::LOST) {
        if (!has_detection_) {
            return;
        }
        RCLCPP_INFO(logger_,
                    "Fresh detection of \"%s\" while lost; resuming "
                    "tracking.",
                    target_class_.c_str());
    }

    if (!camera_info_) {
        state_ = TrackerState::INACTIVE;
        RCLCPP_DEBUG(logger_,
                     "No camera info received, skipping tracking ...");
        return;
    }

    if (!has_first_detection_) {
        state_ = TrackerState::INACTIVE;
        RCLCPP_DEBUG(logger_,
                     "No valid initial target detection received, skipping "
                     "tracking ...");
        return;
    }

    if (has_tracked_target_ && !has_detection_) {
        state_ = TrackerState::LIDAR_ONLY_TRACKING;
        RCLCPP_DEBUG(logger_,
                     "No camera target detection available, falling back to "
                     "LiDAR-only tracking ...");
    } else {
        state_ = TrackerState::FULL_TRACKING;
        RCLCPP_DEBUG(logger_, "Setting tracker to full tracking ...");
    }

    // Note: point cloud availability is no longer required. The tracking
    // measurement comes from the obstacle detector MarkerArray, not from
    // the internal PCL pipeline.

    if (params_.sync.enabled) {
        // When sync is enabled use callback time to detect stale detections.
        // The point-cloud-timestamp path is removed because the point cloud
        // is no longer part of the tracking measurement loop.
        if (params_.sync.use_callback_time) {
            const double detection_skew =
                node_->get_clock()->now().nanoseconds() / 1.0e9 -
                last_valid_detection_callback_time_.nanoseconds() / 1.0e9;
            if (detection_skew > params_.sync.max_detection_skew) {
                if (state_ != TrackerState::LIDAR_ONLY_TRACKING) {
                    RCLCPP_WARN(logger_,
                                "Last detection is too old %.2lf s, switching "
                                "to LiDAR-only tracking ...",
                                detection_skew);
                    state_ = TrackerState::LIDAR_ONLY_TRACKING;
                }
                has_detection_ = false;
            }
        }
    }

    // Tracking measurement via obstacle detector bounding box markers.
    // The LiDAR obstacle detector runs its own clustering and tracking
    // pipeline and produces bounding boxes as a MarkerArray. We use the
    // marker positions as LiDAR measurements, replacing the internal PCL
    // clustering pipeline.

    if (state_ == TrackerState::LIDAR_ONLY_TRACKING) {
        // In LIDAR_ONLY mode we already have an associated obstacle ID from
        // a previous FULL_TRACKING cycle. Find that marker in the latest
        // MarkerArray and use its position as the measurement.
        if (tracked_obstacle_id_ < 0 || !context.has_obstacle_markers) {
            RCLCPP_WARN(logger_,
                        "LIDAR_ONLY: no tracked obstacle ID (%d) or no "
                        "obstacle markers received yet.",
                        tracked_obstacle_id_);
            has_detection_ = false;
            CheckTargetTimeout();
            return;
        }

        bool found = false;
        const auto marker_frame = context.obstacle_markers->markers[0].header.frame_id;

        for (const auto& marker : context.obstacle_markers->markers) {
            if (marker.action ==
                visualization_msgs::msg::Marker::DELETEALL) {
                continue;
            }
            if (marker.id != tracked_obstacle_id_) {
                continue;
            }

            // Transform marker position from its native frame to camera frame
            // so that the EstimatorTick can handle it uniformly.
            const Eigen::Vector3d marker_pos = core::ToEigen(marker.pose.position);
            bounding_box_centroid_ = coord_transformer_.Transform(
                marker.header.frame_id, frame_ids_.Camera(),
                marker_pos);

            bounding_box_size_ = core::ToEigen(marker.scale);
            object_size_ = bounding_box_size_;
            bounding_box_orientation_ = core::ToEigen(marker.pose.orientation);

            has_new_measurement_ = true;
            has_had_first_lidar_measurement_ = true;
            has_tracked_target_ = true;
            last_valid_target_time_ = node_->get_clock()->now();
            last_lidar_seen_time_ = last_valid_target_time_;
            // Record world-frame position for re-acquisition after a brief
            // drop-out: if the obstacle disappears and reappears with a new
            // ID within lidar_reacquire_max_time, we match it by proximity.
            last_lidar_world_pos_ = coord_transformer_.Transform(
                marker_frame, settings_.frames.world_frame,
                marker_pos);
            found = true;
            // JN Improve -->
            double old_yaw;
            if (heading_held_) {
                double old_yaw = last_reliable_yaw_;
                current_yaw_info_ = 1 / (1 / yaw_info_ + settings_.filter.process_variance); // process variance dilutes info
            }
            else {
                double old_yaw = filter_->GetYaw();
                current_yaw_info_ = 1 / filter_->GetFusedYawVariance();
            }

            if (current_yaw_info_ > (0.11)) // try improved pose (0.1 was 1 / (3.14*3.14 / 142))
            {
                const auto q_w2b = coord_transformer_.LookupRotation(
                    marker_frame, settings_.frames.world_frame);
                Eigen::Vector3d euler = q_w2b->toRotationMatrix().eulerAngles(0, 1, 2);
                float cloud_yaw = euler(2);
                pcl::CropBox<pcl::PointXYZ> cropboxFilter(true);
                Eigen::Vector4f min_pt(-bounding_box_size_.x()/2.0, -bounding_box_size_.y() / 2.0, -bounding_box_size_.z() / 2.0, 1.0);
                Eigen::Vector4f max_pt(bounding_box_size_.x() / 2.0, bounding_box_size_.y() / 2.0, bounding_box_size_.z() / 2.0, 1.0);

                Eigen::Vector3d translation_d = coord_transformer_.Transform(
                    settings_.frames.camera_frame, settings_.frames.world_frame, bounding_box_centroid_);
                Eigen::Vector3f translation = marker_pos.cast<float>(); // translation_d.
                cropboxFilter.setInputCloud(context.current_cluster);

                cropboxFilter.setTranslation(translation);
				Eigen::Vector3d bbox_eulerd = bounding_box_orientation_.toRotationMatrix().eulerAngles(0, 1, 2);
				Eigen::Vector3f bbox_euler = bbox_eulerd.cast<float>();
				cropboxFilter.setRotation(bbox_euler);
                cropboxFilter.setMin(min_pt);
                cropboxFilter.setMax(max_pt);
                pcl::PointCloud<pcl::PointXYZ>::Ptr best_cluster(new pcl::PointCloud<pcl::PointXYZ>);
                cropboxFilter.filter(*best_cluster);
                Eigen::Vector3d bounding_box_lidar = coord_transformer_.Transform(
                    settings_.frames.camera_frame, 
                    marker_frame, bounding_box_centroid_); //let ImprovePoseMeasurement work in marker-space
                Eigen::Vector3d improved_centroid;
                double improved_yaw(old_yaw);
                Eigen::Matrix3d R_improved;
                R_improved = ObjectTracker::ImprovePoseMeasurement(best_cluster,
                    bounding_box_lidar,
					marker_frame, settings_.frames.camera_frame, old_yaw, cloud_yaw, improved_yaw);

                bounding_box_orientation_ = Eigen::Quaterniond(
                    cos(improved_yaw / 2), 0.0,
                    0.0, sin(improved_yaw / 2));
                yaw_info_ = current_yaw_info_ + 100.0 / R_improved(2, 2);
                last_reliable_yaw_ = (100.0 / R_improved(2, 2) * improved_yaw + current_yaw_info_ * old_yaw) /
                    (yaw_info_);
                last_lidar_world_pos_ = coord_transformer_.Transform(
                    marker_frame,
                    settings_.frames.world_frame, bounding_box_centroid_);// improved_centroid);
                RCLCPP_INFO(logger_,
                    "LIDAR_ONLY: improved position  "
                    "(%.2f, %.2f, %.2f) camera-frame.",
                    bounding_box_centroid_.x(),
                    bounding_box_centroid_.y(),
                    bounding_box_centroid_.z());
            }
            else {
                last_lidar_world_pos_ = coord_transformer_.Transform(
                    marker_frame,
                    settings_.frames.world_frame, marker_pos);
            if (current_yaw_info_ > (0.11)) // try improved pose (0.1 was 1 / (3.14*3.14 / 142))
            {
                const auto q_w2b = coord_transformer_.LookupRotation(
                    marker_frame, settings_.frames.world_frame);
                Eigen::Vector3d euler = q_w2b->toRotationMatrix().eulerAngles(0, 1, 2);
                float cloud_yaw = euler(2);
                pcl::CropBox<pcl::PointXYZ> cropboxFilter(true);
                Eigen::Vector4f min_pt(-bounding_box_size_.x()/2.0, -bounding_box_size_.y() / 2.0, -bounding_box_size_.z() / 2.0, 1.0);
                Eigen::Vector4f max_pt(bounding_box_size_.x() / 2.0, bounding_box_size_.y() / 2.0, bounding_box_size_.z() / 2.0, 1.0);

                Eigen::Vector3d translation_d = coord_transformer_.Transform(
                    settings_.frames.camera_frame, settings_.frames.world_frame, bounding_box_centroid_);
                Eigen::Vector3f translation = marker_pos.cast<float>(); // translation_d.
                cropboxFilter.setInputCloud(context.current_cluster);

                cropboxFilter.setTranslation(translation);
				Eigen::Vector3d bbox_eulerd = bounding_box_orientation_.toRotationMatrix().eulerAngles(0, 1, 2);
				Eigen::Vector3f bbox_euler = bbox_eulerd.cast<float>();
				cropboxFilter.setRotation(bbox_euler);
                cropboxFilter.setMin(min_pt);
                cropboxFilter.setMax(max_pt);
                pcl::PointCloud<pcl::PointXYZ>::Ptr best_cluster(new pcl::PointCloud<pcl::PointXYZ>);
                cropboxFilter.filter(*best_cluster);
                Eigen::Vector3d bounding_box_lidar = coord_transformer_.Transform(
                    settings_.frames.camera_frame, 
                    marker_frame, bounding_box_centroid_); //let ImprovePoseMeasurement work in marker-space
                Eigen::Vector3d improved_centroid;
                double improved_yaw(old_yaw);
                Eigen::Matrix3d R_improved;
                R_improved = ObjectTracker::ImprovePoseMeasurement(best_cluster,
                    bounding_box_lidar,
					marker_frame, settings_.frames.camera_frame, old_yaw, cloud_yaw, improved_yaw);

                bounding_box_orientation_ = Eigen::Quaterniond(
                    cos(improved_yaw / 2), 0.0,
                    0.0, sin(improved_yaw / 2));
                yaw_info_ = current_yaw_info_ + 100.0 / R_improved(2, 2);
                last_reliable_yaw_ = (100.0 / R_improved(2, 2) * improved_yaw + current_yaw_info_ * old_yaw) /
                    (yaw_info_);
                last_lidar_world_pos_ = coord_transformer_.Transform(
                    marker_frame,
                    settings_.frames.world_frame, bounding_box_centroid_);// improved_centroid);
                RCLCPP_INFO(logger_,
                    "LIDAR_ONLY: improved position  "
                    "(%.2f, %.2f, %.2f) camera-frame.",
                    bounding_box_centroid_.x(),
                    bounding_box_centroid_.y(),
                    bounding_box_centroid_.z());
            }
            else {
                last_lidar_world_pos_ = coord_transformer_.Transform(
                    marker_frame,
                    settings_.frames.world_frame, marker_pos);
                RCLCPP_INFO(logger_,
                    "LIDAR_ONLY: not improved because current_yaw_info_ =  %.2f not > %.2f ", current_yaw_info_, (1 / (3.14 * 3.14 / 142)));
            }
            // <-- JN Improve 
                RCLCPP_INFO(logger_,
                    "LIDAR_ONLY: not improved because current_yaw_info_ =  %.2f not > %.2f ", current_yaw_info_, (1 / (3.14 * 3.14 / 142)));
            }
            // <-- JN Improve 
                        "LIDAR_ONLY: tracking obstacle ID %d at "
                        "(%.2f, %.2f, %.2f) camera-frame.",
                        tracked_obstacle_id_,
                        bounding_box_centroid_.x(),
                        bounding_box_centroid_.y(),
                        bounding_box_centroid_.z());
            break;
        }

        if (!found) {
            // The tracked obstacle is not in the latest markers.
            // Before giving up, attempt re-acquisition: if the elapsed time
            // since the obstacle was last seen is within lidar_reacquire_max_time,
            // search all current markers for one whose world-frame position is
            // within lidar_reacquire_max_dist of the last known position.
            // If found, adopt its ID — the LiDAR briefly lost the obstacle and
            // reassigned a new ID when it reappeared.
            const double elapsed_since_seen =
                (node_->get_clock()->now() - last_lidar_seen_time_).seconds();

            if (elapsed_since_seen <
                params_.tracking.lidar_reacquire_max_time) {
                int reacquire_id = -1;
                double best_dist = params_.tracking.lidar_reacquire_max_dist;
                Eigen::Vector3d best_cam_pos;
                Eigen::Vector3d best_size;
                Eigen::Quaterniond best_quat = Eigen::Quaterniond::Identity();

                for (const auto& m : context.obstacle_markers->markers) {
                    if (m.action == visualization_msgs::msg::Marker::DELETEALL)
                        continue;
                    if (m.id == tracked_obstacle_id_)
                        continue;  // already checked above, not present
                    const Eigen::Vector3d mpos = core::ToEigen(m.pose.position);
                    const Eigen::Vector3d mpos_world = coord_transformer_.Transform(
                        m.header.frame_id, frame_ids_.Map(), mpos);
                    const double d = (mpos_world - last_lidar_world_pos_).norm();
                    if (d < best_dist) {
                        best_dist = d;
                        reacquire_id = m.id;
                        best_cam_pos = coord_transformer_.Transform(
                            m.header.frame_id, frame_ids_.Camera(),
                            mpos);
                        best_size = core::ToEigen(m.scale);
                        best_quat = core::ToEigen(m.pose.orientation);
                    }
                }

                if (reacquire_id >= 0) {
                    RCLCPP_INFO(logger_,
                                "LIDAR_ONLY: re-acquired obstacle as new ID %d "
                                "(was %d, %.2f m away, %.2f s gap).",
                                reacquire_id, tracked_obstacle_id_,
                                best_dist, elapsed_since_seen);
                    tracked_obstacle_id_ = reacquire_id;
                    bounding_box_centroid_ = best_cam_pos;
                    bounding_box_size_ = best_size;
                    object_size_ = best_size;
                    bounding_box_orientation_ = best_quat;
                    has_new_measurement_ = true;
                    has_had_first_lidar_measurement_ = true;
                    has_tracked_target_ = true;
                    last_valid_target_time_ = node_->get_clock()->now();
                    last_lidar_seen_time_ = last_valid_target_time_;
                    last_lidar_world_pos_ = coord_transformer_.Transform(
                        frame_ids_.Camera(),
                        frame_ids_.Map(), best_cam_pos);
                    // Skip further processing — measurement is ready.
                } else {
                    RCLCPP_WARN(logger_,
                                "LIDAR_ONLY: obstacle ID %d not present in latest "
                                "markers; waiting for it to reappear (%.2f s elapsed).",
                                tracked_obstacle_id_, elapsed_since_seen);
                    has_detection_ = false;
                    CheckTargetTimeout();
                    return;
                }
            } else {
                RCLCPP_WARN(logger_,
                            "LIDAR_ONLY: obstacle ID %d not present in latest "
                            "markers; waiting for it to reappear.",
                            tracked_obstacle_id_);
                has_detection_ = false;
                CheckTargetTimeout();
                return;
            }
        }

    } else if (state_ == TrackerState::FULL_TRACKING) {
        // When the camera detects the target, associate the obstacle detector
        // marker that projects closest to the detection bounding-box center
        // in the camera image. This approach is independent of the camera
        // range estimate (which is noisy because it relies on the assumed
        // target height), so it remains robust even when the 3D position
        // estimate is inaccurate.
        if (!context.has_obstacle_markers) {
            RCLCPP_WARN(logger_,
                        "FULL_TRACKING: no obstacle markers received yet, "
                        "falling back to camera-only tracking.");
            CameraCentroidEstimate();
            state_ = TrackerState::CAMERA_ONLY_TRACKING;
            has_detection_ = true;
            has_tracked_target_ = true;
            last_valid_target_time_ = node_->get_clock()->now();
            CheckTargetTimeout();
            MaybePublishContactUpdate();
            return;
        }

        // Detection bounding-box center in pixel coordinates.
        const double det_u =
            detections_message_.bbox.center.position.x;
        const double det_v =
            detections_message_.bbox.center.position.y;

        // Maximum allowed pixel distance: the half-diagonal of the detection
        // bounding box, scaled by obstacle_association_max_dist. Using
        // obstacle_association_max_dist = 1.0 means the marker's projection
        // must land within one half-diagonal of the bbox center. Increase it
        // to be more permissive.
        const double bbox_half_diag = 0.5 * std::sqrt(
            detections_message_.bbox.size_x *
            detections_message_.bbox.size_x +
            detections_message_.bbox.size_y *
            detections_message_.bbox.size_y);
        const double max_pixel_dist =
            params_.tracking.obstacle_association_max_dist * bbox_half_diag;

        double best_pixel_dist = max_pixel_dist;
        int best_id = -1;
        Eigen::Vector3d best_pos_cam;
        Eigen::Vector3d best_size;
        Eigen::Quaterniond best_quat = Eigen::Quaterniond::Identity();
        const auto marker_frame = context.obstacle_markers->markers[0].header.frame_id;

        for (const auto& marker : context.obstacle_markers->markers) {
            if (marker.action ==
                visualization_msgs::msg::Marker::DELETEALL) {
                continue;
            }

            // Transform marker position to camera frame.
            const Eigen::Vector3d pos = core::ToEigen(marker.pose.position);
            const Eigen::Vector3d pos_cam = coord_transformer_.Transform(
                marker.header.frame_id, frame_ids_.Camera(), pos);

            // Skip markers behind the camera.
            if (pos_cam.z() <= 0.0) continue;

            // Project to image pixel coordinates using camera intrinsics.
            const double fx = camera_info_->k[0];
            const double fy = camera_info_->k[4];
            const double cx = camera_info_->k[2];
            const double cy = camera_info_->k[5];
            const double proj_u = fx * pos_cam.x() / pos_cam.z() + cx;
            const double proj_v = fy * pos_cam.y() / pos_cam.z() + cy;

            // Skip projections outside the image.
            if (proj_u < 0.0 ||
                proj_u > static_cast<double>(camera_info_->width) ||
                proj_v < 0.0 ||
                proj_v > static_cast<double>(camera_info_->height)) {
                continue;
            }

            const double pixel_dist = std::sqrt(
                (proj_u - det_u) * (proj_u - det_u) +
                (proj_v - det_v) * (proj_v - det_v));

            if (pixel_dist < best_pixel_dist) {
                best_pixel_dist = pixel_dist;
                best_id = marker.id;
                best_pos_cam = pos_cam;
                best_size = core::ToEigen(marker.scale);
                best_quat = core::ToEigen(marker.pose.orientation);
            }
        }

        if (best_id < 0) {
            // No obstacle marker projects near the camera detection bbox.
            // Fall back to the camera range estimate until the obstacle
            // detector picks up the target again.
            RCLCPP_INFO(logger_,
                        "FULL_TRACKING: no obstacle marker projects within "
                        "%.0f px of detection center (%.0f, %.0f); "
                        "using camera-only measurement.",
                        max_pixel_dist, det_u, det_v);
            CameraCentroidEstimate();
            state_ = TrackerState::CAMERA_ONLY_TRACKING;
            has_detection_ = true;
            has_tracked_target_ = true;
            last_valid_target_time_ = node_->get_clock()->now();
            CheckTargetTimeout();
            MaybePublishContactUpdate();
            return;
        }

        // Associate this obstacle with the target.
        tracked_obstacle_id_ = best_id;
        bounding_box_centroid_ = best_pos_cam;
        bounding_box_size_ = best_size;
        object_size_ = best_size;
        bounding_box_orientation_ = best_quat;

        double old_yaw;
        if (heading_held_) {
            old_yaw = last_reliable_yaw_;
            current_yaw_info_ = yaw_info_;
        }
        else {
            old_yaw = filter_->GetYaw();
            current_yaw_info_ = 1 / filter_->GetFusedYawVariance();
        }

        if (current_yaw_info_ > (0.11)) // try improved pose (0.1 was 1 / (3.14*3.14 / 142))
        {
            const auto q_w2b = coord_transformer_.LookupRotation(
                marker_frame, settings_.frames.world_frame);
            //Eigen::Quaternionf q_w2b = FrameOrientation(marker_frame, settings_.frames.world_frame);
            Eigen::Vector3d euler = q_w2b->toRotationMatrix().eulerAngles(0, 1, 2);
            float cloud_yaw = euler(2);
            pcl::CropBox<pcl::PointXYZ> cropboxFilter(true);
			Eigen::Vector4f min_pt(- bounding_box_size_.x() / 2.0, - bounding_box_size_.y() / 2.0, - bounding_box_size_.z() / 2.0, 1.0);
			Eigen::Vector4f max_pt(bounding_box_size_.x() / 2.0, bounding_box_size_.y() / 2.0, bounding_box_size_.z() / 2.0, 1.0);
			// Eigen::Vector4f min_pt(-6.0, -6.0, -1.0, 1.0);
			// Eigen::Vector4f max_pt(6.0, 6.0, 6.0, 6.0);
            Eigen::Vector3d translation_d = coord_transformer_.Transform(
                settings_.frames.camera_frame,
                settings_.frames.world_frame, bounding_box_centroid_);
            //Eigen::Vector3d translation_d = TransformToCoordinates(
            //    camera_frame_, marker_frame, bounding_box_centroid_);
            Eigen::Vector3f translation = translation_d.cast<float>();
            cropboxFilter.setInputCloud(context.current_cluster);
            cropboxFilter.setTranslation(translation);
            cropboxFilter.setMin(min_pt);
            cropboxFilter.setMax(max_pt);
            pcl::PointCloud<pcl::PointXYZ>::Ptr best_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            cropboxFilter.filter(*best_cluster);
            Eigen::Vector3d best_pos_cam_marker = coord_transformer_.Transform(
                settings_.frames.camera_frame, marker_frame, best_pos_cam); //let ImprovePoseMeasurement work in marker-space
            double improved_yaw = old_yaw;
            Eigen::Matrix3d R_improved;
            R_improved = ObjectTracker::ImprovePoseMeasurement(best_cluster,
                best_pos_cam_marker,
				marker_frame, settings_.frames.camera_frame, old_yaw, cloud_yaw, improved_yaw);

            yaw_info_ = current_yaw_info_ + 1.0 / R_improved(2, 2);
            last_reliable_yaw_ = (1.0 / R_improved(2, 2) * improved_yaw + current_yaw_info_ * old_yaw) /
                (yaw_info_);

            if (R_improved(2, 2) < 3 * 3) {
                bounding_box_orientation_ = Eigen::Quaterniond(
                    cos(improved_yaw / 2), 0.0,
                    0.0, sin(improved_yaw / 2));
                best_pos_cam = bounding_box_centroid_; //bounding_box_centroid_ already updated as sideffect 

            }
        }


        // On the first LiDAR lock-on, reset the Kalman filter so it
        // re-initializes from the reliable LiDAR position rather than
        // continuing from the noisy camera-only state that preceded it.
        // Subsequent LiDAR updates carry on from the already-initialized filter.
        if (!has_had_first_lidar_measurement_) {
            RCLCPP_INFO(logger_,
                        "FULL_TRACKING: first LiDAR lock on obstacle ID %d; "
                        "re-initializing filter from LiDAR position.",
                        best_id);
            filter_initialized_ = false;
        }

        has_new_measurement_ = true;
        has_had_first_lidar_measurement_ = true;
        has_tracked_target_ = true;
        last_valid_target_time_ = node_->get_clock()->now();
        last_lidar_seen_time_ = last_valid_target_time_;
        last_lidar_world_pos_ = coord_transformer_.Transform(
            frame_ids_.Camera(), frame_ids_.Map(),
            best_pos_cam);

        RCLCPP_DEBUG(logger_,
                    "FULL_TRACKING: associated obstacle ID %d "
                    "(projected %.0f px from detection center).",
                    tracked_obstacle_id_, best_pixel_dist);
    }

    has_detection_ = false;

    // Check for tracking timeout across all tracking modes (FULL_TRACKING,
    // LIDAR_ONLY_TRACKING, and CAMERA_ONLY_TRACKING). This ensures that if
    // no valid measurement is obtained for longer than target_timeout, the
    // tracker transitions to NO_DETECTION state.
    CheckTargetTimeout();

    MaybePublishContactUpdate();
}

void ObjectTracker::EstimatorTick() {
    // Skip the target state estimation if no valid target detection has
    // been performed since the last reset.
    if (!has_first_detection_)
        return;

    // Reset the target state estimator if the target is currently being
    // tracked but no valid detections have been received during the timeout
    // window.
    // In LIDAR_ONLY_TRACKING the camera may be permanently lost, so the
    // timeout is measured from the last successful LiDAR cluster rather
    // than the last camera detection. This allows LiDAR-only tracking to
    // continue for as long as the LiDAR keeps finding the target.
    const rclcpp::Time& timeout_reference =
        (state_ == TrackerState::LIDAR_ONLY_TRACKING)
            ? last_valid_target_time_
            : last_valid_detection_time_;
    if ((node_->get_clock()->now() - timeout_reference).seconds() >
        params_.tracking.target_timeout) {
        if (state_ == TrackerState::LIDAR_ONLY_TRACKING ||
            state_ == TrackerState::NO_DETECTION) {
            RCLCPP_WARN(logger_,
                        "Target timeout, resetting estimator ...");
            filter_->SetInitialPosition(Eigen::Vector<double, 3>::Zero());
            filter_->SetInitialVelocity(Eigen::Vector<double, 3>::Zero());
            filter_->ResetCovariance();
            state_ = TrackerState::INACTIVE;
            has_detection_ = false;
            has_first_detection_ = false;
            filter_initialized_ = false;
            has_had_first_lidar_measurement_ = false;
            tracked_obstacle_id_ = -1;
        }
    }

    if (!filter_initialized_) {
        filter_->SetInitialPosition(coord_transformer_.Transform(
            frame_ids_.Camera(), frame_ids_.Map(),
            bounding_box_centroid_));
        filter_->SetInitialVelocity(Eigen::Vector<double, 3>::Zero());
        filter_->ResetCovariance();
        filter_initialized_ = true;
    }

    // Do not advance the filter when there is no active detection.
    // Without a measurement to constrain the velocity, forward integration
    // would cause the position estimate to drift away from the last known
    // position indefinitely.
    if (state_ == TrackerState::NO_DETECTION) {
        if (params_.publish.odometry)     PublishOdometry();
        if (params_.publish.detection_3d) PublishDetection3D();
        return;
    }

    // Freeze the filter if no valid LiDAR measurement has arrived within
    // two tracking periods. A state-based check alone is unreliable because
    // the tracking timer (10 Hz) and estimator timer (20 Hz) race each other:
    // the estimator can read a stale FULL_TRACKING state between the moment
    // the tracking timer starts its tick and the moment it discovers there
    // is no new data and writes NO_DETECTION. The time-based check is immune
    // to this race. It also handles CAMERA_ONLY_TRACKING, where camera range
    // estimates are too noisy to sustain reliable velocity integration.
    const double dt_since_lidar =
        (node_->get_clock()->now() - last_valid_target_time_).seconds();
    if (has_tracked_target_ &&
        dt_since_lidar > 2.0 / params_.tracking.tracking_rate) {
        if (params_.publish.odometry)     PublishOdometry();
        if (params_.publish.detection_3d) PublishDetection3D();
        return;
    }

    // Run the "Predict" step of the Kalman filter.
    filter_->Predict();

    // Check if a new measurement is available to be parsed, then provide a
    // matching measurement vector z_n.
    if (has_new_measurement_) {
        bounding_box_centroid_global_ = coord_transformer_.Transform(
            frame_ids_.Camera(), frame_ids_.Map(),
            bounding_box_centroid_);

        // The IMM measurement vector is 3D [x, y, z].  Velocity and
        // acceleration are estimated internally by each sub-filter.
        Eigen::Matrix<double, 3, 1> measurement_vector;
        measurement_vector(0) = bounding_box_centroid_global_.x();
        measurement_vector(1) = bounding_box_centroid_global_.y();
        measurement_vector(2) = bounding_box_centroid_global_.z();
        if (state_ == TrackerState::CAMERA_ONLY_TRACKING) {
            const std::optional<Eigen::Quaterniond> camera_to_world_rotation =
                coord_transformer_.LookupRotation(
                    frame_ids_.Camera(),
                    frame_ids_.Map());
            if (camera_to_world_rotation) {
                Eigen::Matrix3d RotMatrix =
                    camera_to_world_rotation->toRotationMatrix();
                Eigen::Matrix3d R = RotMatrix * R_rdf_ * RotMatrix.transpose();

                // Run the IMM update step
                // with custom R. if chi2 is acceptable
                double chi2 = filter_->GetChi2IMM2D(measurement_vector, R);
                // Hard 4-sigma treshold TODO open up for soft and as parameter
                if (chi2 < 4 * 4)
                    filter_->Update(measurement_vector, R);
                else {
                    const auto state_filtered = filter_->GetState();

                    RCLCPP_WARN(logger_,
                        "chi2 test failed chi2 at %.2lf z(0)-x = %.2lf, z(1)-y = %.2lf current detection, skipping ...",
                        chi2, measurement_vector(0) - state_filtered(0), measurement_vector(1) - state_filtered(3));
                    state_ = TrackerState::NO_DETECTION;
                }
            }
        }
        else {
            // Run the IMM update step.  if chi2 is acceptable
            double chi2 = filter_->GetChi2IMM2D(measurement_vector);
            // Hard 4-sigma treshold TODO open up for soft and as parameter
            if (chi2 < 4 * 4)
                filter_->Update(measurement_vector);
            else {
                const auto state_filtered = filter_->GetState();

                RCLCPP_WARN(logger_,
                    "chi2 test failed chi2 at %.2lf z(0)-x = %.2lf, z(1)-y = %.2lf current detection, skipping ...",
                    chi2, measurement_vector(0) - state_filtered(0), measurement_vector(1) - state_filtered(3));
                state_ = TrackerState::NO_DETECTION;
            }
        }

        // Mark the latest measurement as processed.
        has_new_measurement_ = false;
    }

    // Get the filtered state, transform it to the world reference frame and
    // publish a matching odometry message.
    const auto state_filtered = filter_->GetState();
    bounding_box_centroid_filtered_.x() = state_filtered(0);
    bounding_box_centroid_filtered_.y() = state_filtered(3);
    bounding_box_centroid_filtered_.z() = state_filtered(6);

    if (params_.publish.odometry) {
        PublishOdometry();
    }
    if (params_.publish.detection_3d) {
        PublishDetection3D();
    }
}

void ObjectTracker::IngestDetection(
    const vision_msgs::msg::Detection2D& detection,
    const rclcpp::Time& header_stamp,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) {
    // Reject the detection if the bounding box touches any image edge.
    // A clipped bbox produces a biased centroid estimate because part of the
    // object is outside the frame; the range/bearing estimate from
    // ConvertBBoxCoordinatesToPoseCentroid_rdf becomes unreliable.
    if (camera_info) {
        const auto& bbox = detection.bbox;
        const double left   = bbox.center.position.x - bbox.size_x / 2.0;
        const double right  = bbox.center.position.x + bbox.size_x / 2.0;
        const double top    = bbox.center.position.y - bbox.size_y / 2.0;
        const double bottom = bbox.center.position.y + bbox.size_y / 2.0;
        if (left <= 0.0 ||
            right  >= static_cast<double>(camera_info->width) ||
            top    <= 0.0 ||
            bottom >= static_cast<double>(camera_info->height)) {
            RCLCPP_DEBUG(logger_,
                         "Bounding box touches image edge (l=%.1f r=%.1f "
                         "t=%.1f b=%.1f img=%ux%u) — skipping camera update.",
                         left, right, top, bottom,
                         camera_info->width,
                         camera_info->height);
            has_detection_ = false;
            return;
        }
    }

    // Store the vision_msgs/msg/Detection2D message, keep track of its
    // timestamp and mark detections as received.
    detections_message_ = detection;
    detection_score_ = detection.results[0].hypothesis.score;
    last_valid_detection_time_ = header_stamp;
    last_valid_detection_callback_time_ = node_->get_clock()->now();

    has_first_detection_ = true;
    has_detection_ = true;
}

void ObjectTracker::MarkDetectionMiss() {
    has_detection_ = false;
}

void ObjectTracker::Reset() {
    filter_->SetInitialPosition(Eigen::Vector3d::Zero());
    filter_->SetInitialVelocity(Eigen::Vector3d::Zero());
    filter_->ResetCovariance();
    filter_initialized_ = false;
    has_first_detection_ = false;
    ResetTrackingState(Eigen::Vector3d::Zero(), TrackerState::INACTIVE);
}

void ObjectTracker::ResetTrackingState(const Eigen::Vector3d& position,
                                       const TrackerState state) {
    bounding_box_centroid_global_ = position;
    bounding_box_centroid_filtered_ = position;
    // Re-acquisition anchor for LiDAR proximity matching.
    last_lidar_world_pos_ = position;

    state_ = state;
    has_tracked_target_ = false;
    has_detection_ = false;
    has_new_measurement_ = false;
    has_had_first_lidar_measurement_ = false;
    tracked_obstacle_id_ = -1;

    const rclcpp::Time now = node_->get_clock()->now();
    last_valid_target_time_ = now;
    last_lidar_seen_time_ = now;
    last_valid_detection_time_ = now;
    last_valid_detection_callback_time_ = now;
}

void ObjectTracker::UpdateSettings(const ObjectTrackerSettings& params) {
    // Note: the IMM filter and the per-target publishers are not rebuilt on
    // a settings update (parity with the original dynamic reconfigure, which
    // never rebuilt them either).
    params_ = params;
}

void ObjectTracker::CheckTargetTimeout() {
    // Timeout fires only when neither camera nor LiDAR has produced a valid
    // measurement for longer than target_timeout. last_valid_target_time_ is
    // updated by any valid measurement from either source, so this correctly
    // keeps the tracker alive as long as at least one sensor sees the target.
    if ((node_->get_clock()->now() - last_valid_target_time_).seconds() >
        params_.tracking.target_timeout) {
        RCLCPP_WARN(logger_, "Tracker timeout reached.");
        state_ = TrackerState::NO_DETECTION;
        has_tracked_target_ = false;
    }
}

// JN addition for camera detection only tracking
void ObjectTracker::CameraCentroidEstimate() {
    try {
        // bbox already in camera before update
        Eigen::Vector3d camera_centroid_rdf =
            ConvertBBoxCoordinatesToPoseCentroid_rdf(detections_message_,
                                                     camera_info_);
        // Swap to flu
        bounding_box_centroid_ = Eigen::Vector3d(camera_centroid_rdf.x(),
                                                 camera_centroid_rdf.y(),
                                                 camera_centroid_rdf.z());
        has_new_measurement_ = true;
    } catch (...) {
        RCLCPP_INFO_STREAM(logger_, "Camera centroid failed: ");
        has_new_measurement_ = false;
        throw;
    }
}

// JN addition for camera detection only tracking
// use vehicle height to estimate centroid using range from boundingbox height
// expressed in righ-down-front (rdf) frame
// Added to code by Jonas N
Eigen::Vector3d ObjectTracker::ConvertBBoxCoordinatesToPoseCentroid_rdf(
    const vision_msgs::msg::Detection2D& detections_message,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_message) {
    const double car_size_z = params_.camera.target_height;
    double target_z_f = (double)camera_info_message->k[4] / (double)detections_message.bbox.size_y *
        car_size_z;
    double target_x_r = target_z_f / (double)camera_info_message->k[0] *
        (double)(detections_message.bbox.center.position.x - camera_info_message->k[2]);

    double target_y_d = target_z_f / (double)camera_info_message->k[4] *
        (double)(detections_message.bbox.center.position.y - camera_info_message->k[5]);
    Eigen::Vector3d camera_estimated_centroid_rdf(target_x_r, target_y_d, target_z_f);

    // covariance jacobians
    const double s2_pixel = params_.camera.bbox_pixel_sigma *
        params_.camera.bbox_pixel_sigma;
    double s2_forwards = (double)camera_info_message->k[4] / pow((double)detections_message.bbox.size_y, 2) *
        car_size_z * s2_pixel * (double)camera_info_message->k[4] /
            pow((double)detections_message.bbox.size_y, 2) *
        car_size_z;
    double s2_right = target_z_f / (double)camera_info_message->k[0] * s2_pixel *
        target_z_f / (double)camera_info_message->k[0];
    double s2_down = target_z_f / (double)camera_info_message->k[4] * s2_pixel *
        target_z_f / (double)camera_info_message->k[4];

    R_rdf_(0, 0) = std::max(params_.filter.measurement_variance, s2_right);
    R_rdf_(1, 1) = std::max(params_.filter.measurement_variance, s2_down);
    R_rdf_(2, 2) = std::max(params_.filter.measurement_variance, s2_forwards);
    RCLCPP_INFO_STREAM(logger_, "ConvertBBoxCoordinatesToPoseCentroid of size " << detections_message.bbox.size_y << " pixel, " << car_size_z << "m" << '\n'
        << "[x, y ,z] = [ " << target_x_r
        << ", " << target_y_d
        << ", " << target_z_f << "]" << '\n');
    return camera_estimated_centroid_rdf;
}

void ObjectTracker::UpdateHeadingHold() {
    const double speed = filter_->GetCTRSpeed();
    if (!heading_held_ && speed < params_.tracking.heading_min_speed) {
        heading_held_ = true;
    } else if (heading_held_ &&
               speed >= params_.tracking.heading_resume_speed) {
        heading_held_ = false;
    }
    if (!heading_held_) {
        last_reliable_yaw_ = filter_->GetYaw();
    }
}
// JN addition for better pose measurement
// Added to code by Jonas N
// Input: pcl cluster object_cluster from obstacle detector, current yaw is read from global variable (last_reliable_yaw_)
// return covariance matrix (x,y,yaw) and improved_centroid and improved_yaw
Eigen::Matrix3d ObjectTracker::ImprovePoseMeasurement(
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cluster,
    Eigen::Vector3d measured_centroid,
	const std::string& source_frame,
	const std::string& target_frame,
    double current_yaw,
    double platform_yaw,
    double& improved_yaw) {

    bool has_end_plane_ = false;
    bool has_flank_plane_ = false;
	Eigen::Vector3d improved_centroid = measured_centroid; // initialize on cloud centroid
    improved_yaw = current_yaw;
    current_yaw = current_yaw - platform_yaw;
    const double sigma = settings_.filter.measurement_variance;
    const double sigma2 = sigma * sigma;
    Eigen::Matrix3d R(sigma2 * Eigen::Matrix3d::Identity());
    R(2, 2) = 30 * 30; //No improvment
    Eigen::Vector3f heading_perpendicular(-sin(current_yaw), cos(current_yaw), 0.0);
    // end plane is perpendicular to heading
    sac_segmentation_.setEpsAngle((M_PI / 180.0) * 5);
    sac_segmentation_.setAxis(heading_perpendicular);
    sac_segmentation_.setInputCloud(object_cluster);
    sac_segmentation_.setDistanceThreshold(0.4);
    pcl::PointIndices::Ptr inliers_e(new pcl::PointIndices); // _e as in end
    pcl::ModelCoefficients::Ptr coefficients_e(new pcl::ModelCoefficients);
    sac_segmentation_.segment(*inliers_e, *coefficients_e);

    Eigen::Matrix3d covariance_matrix_e;
    Eigen::Vector4d plane_center_e;
    Eigen::Vector4d centroid_f;
    Eigen::Vector4d centroid_e;


    int plane_threshold = 12;
    if (inliers_e->indices.size() > plane_threshold) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_e(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*object_cluster, *inliers_e, *cloud_e);

        pcl::compute3DCentroid(*cloud_e, centroid_e);
        pcl::computeCovarianceMatrix(*object_cluster, *inliers_e, centroid_e, covariance_matrix_e);
        has_end_plane_ = true;

    }
    else {
        has_end_plane_ = false;
    }

    Eigen::Vector3f heading_direction(cos(current_yaw), sin(current_yaw), 0.0);

    pcl::PointIndices::Ptr inliers_f(new pcl::PointIndices);  // _f as in flank
    pcl::ModelCoefficients::Ptr coefficients_f(new pcl::ModelCoefficients);
    sac_segmentation_.setAxis(heading_direction);
    sac_segmentation_.segment(*inliers_f, *coefficients_f);
    Eigen::Matrix3d covariance_matrix_f;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (inliers_f->indices.size() > plane_threshold) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*object_cluster, *inliers_f, *cloud_f);

        pcl::compute3DCentroid(*cloud_f, centroid_f);
        pcl::computeCovarianceMatrix(*object_cluster, *inliers_f, centroid_f, covariance_matrix_f);
        has_flank_plane_ = true;

    }
    else {

        has_flank_plane_ = false;
    }
    if (has_end_plane_) {

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance_matrix_e);
        Eigen::Matrix3d eigenVectors = eigen_solver.eigenvectors();
        eigenVectors.col(2) = eigenVectors.col(0).cross(eigenVectors.col(1));  // ensure right hand matrix
        Eigen::Vector3d eigenValues = eigen_solver.eigenvalues();
        int r_d, c_d;
        double plane_deviation2_e = eigenValues.minCoeff(&r_d, &c_d);    // [s, i] = min(R(:)); [r, c] = ind2sub(size(R), i);
        Eigen::Vector3d n_hat = eigenVectors.col(c_d); // plane normal
        n_hat(2) = 0.0; //vertical plane

        n_hat.normalize();
        Eigen::Vector3d z_hat(0.0, 0.0, 1.0);
        Eigen::Vector3d heading_direction3d = heading_direction.cast <double>();
        double direction_test = n_hat.dot(heading_direction3d); //n_hat mostly in heading_direction
        if (direction_test < 0) n_hat = -n_hat;
        // TODO sanity check on direction_test size not just sign
        Eigen::Vector3d y_hat = z_hat.cross(n_hat);

        Eigen::Vector4d delta(measured_centroid.x() - centroid_e.x(),
            measured_centroid.y() - centroid_e.y(),
            measured_centroid.z() - centroid_e.z(), 1.0);

        double plane_deviation2_f;
        if (has_flank_plane_) // can improve n_hat
        {
 
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver_f(covariance_matrix_f);
            Eigen::Matrix3d eigenVectors_f = eigen_solver_f.eigenvectors();
            eigenVectors_f.col(2) = eigenVectors_f.col(0).cross(eigenVectors_f.col(1));  // ensure right hand matrix
            Eigen::Vector3d eigenValues_f = eigen_solver_f.eigenvalues();
            int r_f, c_f;
            plane_deviation2_f = eigenValues_f.minCoeff(&r_f, &c_f);    // [s, i] = min(R(:)); [r, c] = ind2sub(size(R), i);
            Eigen::Vector3d y_hat = eigenVectors_f.col(c_f); // flank plane normal new y_hat
            y_hat(2) = 0.0; //vertical plane
            y_hat.normalize();
            Eigen::Vector3d z_hat(0.0, 0.0, 1.0);
            Eigen::Vector3d n_hat = y_hat.cross(z_hat);
            double direction_test = n_hat.dot(heading_direction3d); //n_hat mostly in heading_direction
            if (direction_test < 0) {
                n_hat = -n_hat;
                y_hat = -y_hat;
            }        }
        Eigen::Matrix4d endTransform(Eigen::Matrix4d::Identity());
        endTransform.block(0, 0, 1, 3) = n_hat;
        endTransform.block(1, 0, 1, 3) = y_hat;
        endTransform.block(2, 0, 1, 3) = z_hat;

        Eigen::Vector4d delta_b = endTransform.inverse() * delta; // delta in body coordinates
        double rear_to_wheel = 0.0;// 1.5;
        double car_length = 0.0;
        Eigen::Vector3d plane_center(centroid_e.x(), centroid_e.y(), centroid_e.z());
        if (delta_b(0) > 0.0) // rear end since pointcloud centroid in front of plane 
            improved_centroid = plane_center; // +rear_to_wheel * n_hat;
        else //front measured
            improved_centroid = plane_center; // -(car_length - rear_to_wheel) * n_hat;
        improved_yaw = atan2(n_hat(1), n_hat(0));

        // setup clouds for information calculation
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
        double info_plane = 0;

        Eigen::Vector4d centroid; //local centroid
        if (has_flank_plane_) {
            pcl::copyPointCloud(*object_cluster, *inliers_f, *cloud);
            pcl::transformPointCloud(*cloud, *transformed, endTransform);
            pcl::compute3DCentroid(*transformed, centroid);
            double s2 = plane_deviation2_f; // estimated sigma of plane points
            for (std::size_t idx = 0; idx < transformed->points.size(); idx++) {
                info_plane += (transformed->points[idx].x - centroid.x()) * 1 / s2 *
                    (transformed->points[idx].x - centroid.x()); //flank plane along local x-direction 
            }
        }
        else {
            pcl::copyPointCloud(*object_cluster, *inliers_e, *cloud);
            pcl::transformPointCloud(*cloud, *transformed, endTransform);
            pcl::compute3DCentroid(*transformed, centroid);
            double s2 = plane_deviation2_e; // estimated sigma of plane points
            for (std::size_t idx = 0; idx < transformed->points.size(); idx++) {
                info_plane += (transformed->points[idx].y - centroid.y()) * 1 / s2 *
                    (transformed->points[idx].y - centroid.y()); //end plane along local y-direction 
            }
        }
        Eigen::Matrix4d R_b(Eigen::Matrix4d::Identity());
        R_b(0, 0) = 0.2 * 0.2;
        R_b(1, 1) = 0.5 * 0.5;
        improved_yaw = -platform_yaw + improved_yaw;
        if (info_plane > 0) {
            R_b(2, 2) = 1 / info_plane;
            // ugly sideeffect on global because Eigen referencing is beyond me right now
			 bounding_box_centroid_ = coord_transformer_.Transform(
				source_frame,
				target_frame, improved_centroid);
        }
		else {
			R_b(2, 2) = 30 * 30;
		}
       
        Eigen::Matrix4d R4 = endTransform.inverse() * R_b * endTransform;
        R = R4.block(0, 0, 3, 3);

    }
    return R;

    //pcl::PCDWriter::writeASCII(file_name, object_cluster,
    //		origin = Eigen::Vector4f::Zero(), orientation = Eigen::Quaternionf::Identity(), 8)
    // pcl::transformPointCloud(*cloud, *transformed, transformation);

}
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*object_cluster, *inliers_f, *cloud_f);

        pcl::compute3DCentroid(*cloud_f, centroid_f);
        pcl::computeCovarianceMatrix(*object_cluster, *inliers_f, centroid_f, covariance_matrix_f);
        has_flank_plane_ = true;

    }
    else {

        has_flank_plane_ = false;
    }
    if (has_end_plane_) {

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance_matrix_e);
        Eigen::Matrix3d eigenVectors = eigen_solver.eigenvectors();
        eigenVectors.col(2) = eigenVectors.col(0).cross(eigenVectors.col(1));  // ensure right hand matrix
        Eigen::Vector3d eigenValues = eigen_solver.eigenvalues();
        int r_d, c_d;
        double plane_deviation2_e = eigenValues.minCoeff(&r_d, &c_d);    // [s, i] = min(R(:)); [r, c] = ind2sub(size(R), i);
        Eigen::Vector3d n_hat = eigenVectors.col(c_d); // plane normal
        n_hat(2) = 0.0; //vertical plane

        n_hat.normalize();
        Eigen::Vector3d z_hat(0.0, 0.0, 1.0);
        Eigen::Vector3d heading_direction3d = heading_direction.cast <double>();
        double direction_test = n_hat.dot(heading_direction3d); //n_hat mostly in heading_direction
        if (direction_test < 0) n_hat = -n_hat;
        // TODO sanity check on direction_test size not just sign
        Eigen::Vector3d y_hat = z_hat.cross(n_hat);

        Eigen::Vector4d delta(measured_centroid.x() - centroid_e.x(),
            measured_centroid.y() - centroid_e.y(),
            measured_centroid.z() - centroid_e.z(), 1.0);

        double plane_deviation2_f;
        if (has_flank_plane_) // can improve n_hat
        {
 
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver_f(covariance_matrix_f);
            Eigen::Matrix3d eigenVectors_f = eigen_solver_f.eigenvectors();
            eigenVectors_f.col(2) = eigenVectors_f.col(0).cross(eigenVectors_f.col(1));  // ensure right hand matrix
            Eigen::Vector3d eigenValues_f = eigen_solver_f.eigenvalues();
            int r_f, c_f;
            plane_deviation2_f = eigenValues_f.minCoeff(&r_f, &c_f);    // [s, i] = min(R(:)); [r, c] = ind2sub(size(R), i);
            Eigen::Vector3d y_hat = eigenVectors_f.col(c_f); // flank plane normal new y_hat
            y_hat(2) = 0.0; //vertical plane
            y_hat.normalize();
            Eigen::Vector3d z_hat(0.0, 0.0, 1.0);
            Eigen::Vector3d n_hat = y_hat.cross(z_hat);
            double direction_test = n_hat.dot(heading_direction3d); //n_hat mostly in heading_direction
            if (direction_test < 0) {
                n_hat = -n_hat;
                y_hat = -y_hat;
            }        }
        Eigen::Matrix4d endTransform(Eigen::Matrix4d::Identity());
        endTransform.block(0, 0, 1, 3) = n_hat;
        endTransform.block(1, 0, 1, 3) = y_hat;
        endTransform.block(2, 0, 1, 3) = z_hat;

        Eigen::Vector4d delta_b = endTransform.inverse() * delta; // delta in body coordinates
        double rear_to_wheel = 0.0;// 1.5;
        double car_length = 0.0;
        Eigen::Vector3d plane_center(centroid_e.x(), centroid_e.y(), centroid_e.z());
        if (delta_b(0) > 0.0) // rear end since pointcloud centroid in front of plane 
            improved_centroid = plane_center; // +rear_to_wheel * n_hat;
        else //front measured
            improved_centroid = plane_center; // -(car_length - rear_to_wheel) * n_hat;
        improved_yaw = atan2(n_hat(1), n_hat(0));

        // setup clouds for information calculation
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
        double info_plane = 0;

        Eigen::Vector4d centroid; //local centroid
        if (has_flank_plane_) {
            pcl::copyPointCloud(*object_cluster, *inliers_f, *cloud);
            pcl::transformPointCloud(*cloud, *transformed, endTransform);
            pcl::compute3DCentroid(*transformed, centroid);
            double s2 = plane_deviation2_f; // estimated sigma of plane points
            for (std::size_t idx = 0; idx < transformed->points.size(); idx++) {
                info_plane += (transformed->points[idx].x - centroid.x()) * 1 / s2 *
                    (transformed->points[idx].x - centroid.x()); //flank plane along local x-direction 
            }
        }
        else {
            pcl::copyPointCloud(*object_cluster, *inliers_e, *cloud);
            pcl::transformPointCloud(*cloud, *transformed, endTransform);
            pcl::compute3DCentroid(*transformed, centroid);
            double s2 = plane_deviation2_e; // estimated sigma of plane points
            for (std::size_t idx = 0; idx < transformed->points.size(); idx++) {
                info_plane += (transformed->points[idx].y - centroid.y()) * 1 / s2 *
                    (transformed->points[idx].y - centroid.y()); //end plane along local y-direction 
            }
        }
        Eigen::Matrix4d R_b(Eigen::Matrix4d::Identity());
        R_b(0, 0) = 0.2 * 0.2;
        R_b(1, 1) = 0.5 * 0.5;
        improved_yaw = -platform_yaw + improved_yaw;
        if (info_plane > 0) {
            R_b(2, 2) = 1 / info_plane;
            // ugly sideeffect on global because Eigen referencing is beyond me right now
			 bounding_box_centroid_ = coord_transformer_.Transform(
				source_frame,
				target_frame, improved_centroid);
        }
		else {
			R_b(2, 2) = 30 * 30;
		}
       
        Eigen::Matrix4d R4 = endTransform.inverse() * R_b * endTransform;
        R = R4.block(0, 0, 3, 3);

    }
    return R;

    //pcl::PCDWriter::writeASCII(file_name, object_cluster,
    //		origin = Eigen::Vector4f::Zero(), orientation = Eigen::Quaternionf::Identity(), 8)
    // pcl::transformPointCloud(*cloud, *transformed, transformation);

}
void ObjectTracker::PublishOdometry() {
    UpdateHeadingHold();

    nav_msgs::msg::Odometry odometry_message;
    odometry_message.header.stamp = node_->get_clock()->now();
    odometry_message.header.frame_id = frame_ids_.Map();
    odometry_message.child_frame_id = frame_ids_.BaseLink();
    odometry_message.pose.pose.position = core::ToPointMsg(bounding_box_centroid_global_);
    odometry_message.pose.pose.orientation = core::YawToQuaternionMsg(last_reliable_yaw_);

    odometry_publisher_->publish(odometry_message);

    nav_msgs::msg::Odometry tracked_target_message;
    tracked_target_message.header.stamp = node_->get_clock()->now();
    tracked_target_message.header.frame_id = frame_ids_.Map();
    tracked_target_message.child_frame_id = frame_ids_.BaseLink();
    tracked_target_message.pose.pose.position = core::ToPointMsg(bounding_box_centroid_filtered_);
    tracked_target_message.pose.pose.orientation = core::YawToQuaternionMsg(last_reliable_yaw_);
	tracked_target_message.twist.twist.linear = core::ToVector3Msg(filter_->GetVelocity3D());

    const Eigen::Quaterniond world_to_target = core::ToEigen(tracked_target_message.pose.pose.orientation).inverse();
    tracked_target_message.twist.twist.linear = core::ToVector3Msg(world_to_target * filter_->GetVelocity3D());
    Eigen::Matrix<double, 6, 6> TrackedCovariance = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 5, 5> Pt = filter_->GetCTRCovariance();
    TrackedCovariance(0, 0) = Pt(0, 0);  // x variance
    TrackedCovariance(0, 1) = Pt(0, 2);  // xy covariance
    TrackedCovariance(1, 0) = Pt(2, 0);
    TrackedCovariance(1, 1) = Pt(2, 2);  // y variance
    TrackedCovariance(2, 2) = 100.0;     // no information on z
    TrackedCovariance(3, 3) = 9.0;       // no information on roll
    TrackedCovariance(4, 4) = 9.0;       // no information on pitch
    TrackedCovariance(5, 5) = filter_->GetFusedYawVariance();
    tracked_target_message.pose.covariance = core::ToCovarianceMsg(TrackedCovariance);

    // Cache the latest estimate so GetTrackerStatus() can report it (pose +
    // covariance) without recomputing.
    last_tracked_odometry_ = tracked_target_message;

    tracked_target_odometry_publisher_->publish(tracked_target_message);
	leader_odom_publisher_->publish(tracked_target_message);
}

avt_341_msgs::msg::TrackerStatus ObjectTracker::GetTrackerStatus() const {
    avt_341_msgs::msg::TrackerStatus status;
    status.header.stamp = node_->get_clock()->now();
    status.header.frame_id = target_class_;
    status.state = static_cast<uint8_t>(state_);
    status.tracked_object_id = target_class_;
    status.odom_estimate = last_tracked_odometry_;
    return status;
}

void ObjectTracker::PublishDetection3D() {
    vision_msgs::msg::ObjectHypothesisWithPose object_hypothesis_message;

    object_hypothesis_message.hypothesis.class_id = target_class_;
    object_hypothesis_message.hypothesis.score = detection_score_;

    vision_msgs::msg::Detection3D detection_message;

    detection_message.header.stamp = node_->get_clock()->now();
    detection_message.header.frame_id = frame_ids_.Camera();

    detection_message.results.push_back(object_hypothesis_message);

    detection_message.bbox.size = core::ToVector3Msg(bounding_box_size_);
    detection_message.bbox.center.position = core::ToPointMsg(bounding_box_centroid_global_);
    detection_message.bbox.center.orientation = core::ToQuaternionMsg(bounding_box_orientation_);

    detection_publisher_->publish(detection_message);
}

	// JN suggestion for tracker input given acces to filter_ and a function getMu in filter_
	//  mu = filter_.getMu();
	//  P = filter_.getPoseCovariance();
	// if  (mu[2]>0.5 & P(0,0)<2.0 & P(1,1)<2.0) //2.0 and 0.5 should be parameters
	// JN suggestion for tracker input given acces to filter_ and a function getMu in filter_
	//  mu = filter_.getMu();
	//  P = filter_.getPoseCovariance();
	// if  (mu[2]>0.5 & P(0,0)<2.0 & P(1,1)<2.0) //2.0 and 0.5 should be parameters
}  // namespace perception
}  // namespace avt_341_nav
