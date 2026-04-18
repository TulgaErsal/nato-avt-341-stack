/**
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 +                      _    _    _    _    _    _    _                      +
 +                     / \  / \  / \  / \  / \  / \  / \                     +
 +                    ( A )( V )( T )( - )( 3 )( 4 )( 1 )                    +
 +                     \_/  \_/  \_/  \_/  \_/  \_/  \_/                     +
 +       _    _    _    _    _    _    _    _     _    _    _    _    _      +
 +      / \  / \  / \  / \  / \  / \  / \  / \   / \  / \  / \  / \  / \     +
 +     ( A )( U )( T )( O )( N )( O )( M )( Y ) ( S )( T )( A )( C )( K )    +
 +      \_/  \_/  \_/  \_/  \_/  \_/  \_/  \_/   \_/  \_/  \_/  \_/  \_/     +
 +                                                                           +
 +  AVT-341 Autonomy Stack: Autonomous Navigation Stack for Ground Vehicles  +
 +                                                                           +
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

* @file      object_tracking_node.cpp
* @author    Dario Sirangelo (dsi@aarhusrobotics.com)
* @brief     Source file for the camera/LiDAR sensor fusion object tracker
             rclcpp ROS node.
* @copyright
  MIT License

  NATO AVT-341 Autonomy Stack: Autonomous Navigation Stack for Ground Vehicles
  Copyright (c) 2024 Dario Sirangelo (dsi@aarhusrobotics.com)

  NOTE: The above copyright only applies to the contents of this file. The
  source code contained in this file is a direct port from the GitHub repository
  aarhus-robotics/navi, released by the copyright holder under the MIT license.

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include <avt_341/perception/tracking/object_tracking_node.hpp>

namespace avt_341 {
namespace perception {

ObjectTrackingNode::ObjectTrackingNode() : rclcpp::Node("object_tracker") {
    GetParameters();
    Initialize();
    CreateSubscriptions();
    CreateTimers();
    CreateServices();
    CreatePublishers();

    estimator_timer_->reset();
    tracking_timer_->reset();
    info_timer_->reset();
}

void ObjectTrackingNode::GetParameters() {
    declare_parameter("tracking_rate", 10.0);
    tracking_rate_ = get_parameter("tracking_rate").as_double();

    declare_parameter("info_rate", 1.0);
    info_rate_ = get_parameter("info_rate").as_double();

    declare_parameter("camera_frame", "camera_optical");
    camera_frame_ = get_parameter("camera_frame").as_string();

    declare_parameter("frame_prefix", "");
    const std::string frame_prefix = get_parameter("frame_prefix").as_string();

    camera_frame_ = frame_prefix + camera_frame_;

    declare_parameter("world_frame", "Q");
    world_frame_ = get_parameter("world_frame").as_string();

    declare_parameter("odometry_child_frame", "odom");
    odometry_child_frame_ = get_parameter("odometry_child_frame").as_string();

    declare_parameter("filters_downsampling_leaf_size", 0.3);
    leaf_size_ = get_parameter("filters_downsampling_leaf_size").as_double();

    declare_parameter("filters_passthrough_min", 3.0);
    passthrough_distance_min_ =
        get_parameter("filters_passthrough_min").as_double();

    declare_parameter("filters_passthrough_max", 40.0);
    passthrough_distance_max_ =
        get_parameter("filters_passthrough_max").as_double();

    declare_parameter("filters_clustering_tolerance", 0.6);
    clustering_tolerance_ =
        get_parameter("filters_clustering_tolerance").as_double();

    declare_parameter("filters_clustering_size_minimum", 50);
    cluster_size_min_ =
        get_parameter("filters_clustering_size_minimum").as_int();

    declare_parameter("filters_clustering_size_maximum", 500);
    cluster_size_max_ =
        get_parameter("filters_clustering_size_maximum").as_int();

    declare_parameter("filters_clustering_min_height", 1.0);
    cluster_height_min_ =
        get_parameter("filters_clustering_min_height").as_double();

    declare_parameter("filters_clustering_max_height", 4.0);
    cluster_height_max_ =
        get_parameter("filters_clustering_max_height").as_double();

    declare_parameter("filters_clustering_min_width", 0.3);
    cluster_width_min_ =
        get_parameter("filters_clustering_min_width").as_double();

    declare_parameter("filters_clustering_max_width", 4.0);
    cluster_width_max_ =
        get_parameter("filters_clustering_max_width").as_double();

    declare_parameter("filters_clustering_min_depth", 0.3);
    cluster_depth_min_ =
        get_parameter("filters_clustering_min_depth").as_double();

    declare_parameter("filters_clustering_max_depth", 6.0);
    cluster_depth_max_ =
        get_parameter("filters_clustering_max_depth").as_double();

    declare_parameter("filters_clustering_distance_reference", 10.0);
    cluster_distance_ref_ =
        get_parameter("filters_clustering_distance_reference").as_double();

    declare_parameter("filters_ground_max_iterations", 50);
    sac_segmentation_max_iterations_ =
        get_parameter("filters_ground_max_iterations").as_int();

    declare_parameter("filters_ground_threshold", 0.2);
    sac_segmentation_threshold_ =
        get_parameter("filters_ground_threshold").as_double();

    declare_parameter("filters_ground_angle", 3.0);
    sac_segmentation_angle_ = get_parameter("filters_ground_angle").as_double();

    declare_parameter("filters_roi_scale_factor", 1.5);
    roi_scale_factor_ = get_parameter("filters_roi_scale_factor").as_double();

    declare_parameter("filters_kalman_rate", 50.0);
    estimator_rate_ = get_parameter("filters_kalman_rate").as_double();

    declare_parameter("filters_kalman_process", 0.01);
    filter_process_variance_ =
        get_parameter("filters_kalman_process").as_double();

    declare_parameter("filters_kalman_measurement", 1.0);
    filter_measurement_variance_ =
        get_parameter("filters_kalman_measurement").as_double();

    declare_parameter("camera_target_height", 5.0);
    camera_target_height_ = get_parameter("camera_target_height").as_double();

    declare_parameter("camera_bbox_pixel_sigma", 4.0);
    camera_bbox_pixel_sigma_ = get_parameter("camera_bbox_pixel_sigma").as_double();

    declare_parameter("filters_imm_cv_init_prob", 0.33);
    imm_cv_init_prob_ = get_parameter("filters_imm_cv_init_prob").as_double();

    declare_parameter("filters_imm_ctr_init_prob", 0.33);
    imm_ctr_init_prob_ = get_parameter("filters_imm_ctr_init_prob").as_double();

    declare_parameter("filters_imm_nm_init_prob", 0.33);
    imm_nm_init_prob_ = get_parameter("filters_imm_nm_init_prob").as_double();

    declare_parameter("filters_imm_persistence_prob", 0.9);
    imm_persistence_prob_ = get_parameter("filters_imm_persistence_prob").as_double();

    declare_parameter("filters_use_pca_centroid", false);
    use_pca_centroid_ = get_parameter("filters_use_pca_centroid").as_bool();

    declare_parameter("tracker_autostart", true);
    use_autostart_ = get_parameter("tracker_autostart").as_bool();

    declare_parameter("tracker_use_mission_manager", true);
    use_mission_manager_ = get_parameter("tracker_use_mission_manager").as_bool();

    declare_parameter("tracker_target_class", "mrzr4");
    autostart_target_class_ = get_parameter("tracker_target_class").as_string();
    declare_parameter("tracker_timeout", 5.0);
    target_timeout_ = get_parameter("tracker_timeout").as_double();

    declare_parameter("sync_enable", true);
    sync_messages_ = get_parameter("sync_enable").as_bool();

    declare_parameter("sync_use_callback", false);
    use_callback_time_ = get_parameter("sync_use_callback").as_bool();

    declare_parameter("sync_detection", 0.1);
    max_detection_skew_ = get_parameter("sync_detection").as_double();

    declare_parameter("publish_clouds_fov", false);
    publish_fov_cloud_ = get_parameter("publish_clouds_fov").as_bool();

    declare_parameter("publish_clouds_roi", false);
    publish_roi_cloud_ = get_parameter("publish_clouds_roi").as_bool();

    declare_parameter("publish_clouds_ground", false);
    publish_ground_cloud_ = get_parameter("publish_clouds_ground").as_bool();

    declare_parameter("publish_clouds_cluster", false);
    publish_cluster_cloud_ = get_parameter("publish_clouds_cluster").as_bool();

    declare_parameter("publish_clouds_cropbox", false);
    publish_cropbox_cloud_ = get_parameter("publish_clouds_cropbox").as_bool();

    declare_parameter("publish_pose", true);
    publish_pose_ = get_parameter("publish_pose").as_bool();

    declare_parameter("filters_pose", true);
    use_filtered_pose_ = get_parameter("filters_pose").as_bool();

    declare_parameter("publish_odometry", false);
    publish_odometry_ = get_parameter("publish_odometry").as_bool();

    declare_parameter("filters_odometry", true);
    use_filtered_odometry_ = get_parameter("filters_odometry").as_bool();

    declare_parameter("publish_detection", false);
    publish_detection_3d_ = get_parameter("publish_detection").as_bool();

    declare_parameter("publish_image", false);
    publish_image_ = get_parameter("publish_image").as_bool();

    declare_parameter("filters_use_manual_roi", false);
    use_manual_roi_size_ = get_parameter("filters_use_manual_roi").as_bool();

    declare_parameter("obstacle_markers_topic",
                      std::string("/avt_341/lidar_detector/bboxes"));
    obstacle_markers_topic_ =
        get_parameter("obstacle_markers_topic").as_string();

    declare_parameter("obstacle_association_max_dist", 5.0);
    obstacle_association_max_dist_ =
        get_parameter("obstacle_association_max_dist").as_double();

    declare_parameter("lidar_reacquire_max_time", 0.5);
    lidar_reacquire_max_time_ =
        get_parameter("lidar_reacquire_max_time").as_double();

    declare_parameter("lidar_reacquire_max_dist", 3.0);
    lidar_reacquire_max_dist_ =
        get_parameter("lidar_reacquire_max_dist").as_double();

    declare_parameter("filters_manual_roi_size",
                      std::vector<double>{1.0, 1.0, 1.0});
    roi_bounding_box_3d_size_ = Eigen::Vector3d(
        get_parameter("filters_manual_roi_size").as_double_array()[0],
        get_parameter("filters_manual_roi_size").as_double_array()[1],
        get_parameter("filters_manual_roi_size").as_double_array()[2]);

    on_set_parameters_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&ObjectTrackingNode::SetParametersCallback, this,
                  std::placeholders::_1));
}

void ObjectTrackingNode::CreateSubscriptions() {
    // Create the transform listener.
    transform_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    transform_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*transform_buffer_);

    detections_subscription_ =
        create_subscription<vision_msgs::msg::Detection2DArray>(
            "detection_2d", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&ObjectTrackingNode::DetectionsCallback, this,
                      std::placeholders::_1));

    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        "image", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
        std::bind(&ObjectTrackingNode::ImageCallback, this,
                  std::placeholders::_1));

    camera_info_subscription_ =
        create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&ObjectTrackingNode::CameraInfoCallback, this,
                      std::placeholders::_1));

    point_cloud_subscription_ =
        create_subscription<sensor_msgs::msg::PointCloud2>(
            "points/input", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&ObjectTrackingNode::PointCloudCallback, this,
                      std::placeholders::_1));

    if(use_mission_manager_) {
        task_status_subscription_ =
            create_subscription<avt_341_msgs::msg::MissionTaskStatus>(
                "task", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
                std::bind(&ObjectTrackingNode::TaskStatusCallback, this,
                        std::placeholders::_1));
    }

    obstacle_markers_subscription_ =
        create_subscription<visualization_msgs::msg::MarkerArray>(
            obstacle_markers_topic_, RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            std::bind(&ObjectTrackingNode::ObstacleMarkersCallback, this,
                      std::placeholders::_1));
}

void ObjectTrackingNode::CreateServices() {
    if (!use_mission_manager_) {
        set_target_service_server_ =
            create_service<avt_341_msgs::srv::SetTarget>(
                "set_target",
                std::bind(&ObjectTrackingNode::SetTargetServiceCallback, this,
                          std::placeholders::_1, std::placeholders::_2));
    }
}

void ObjectTrackingNode::CreateTimers() {
    estimator_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / estimator_rate_),
        std::bind(&ObjectTrackingNode::EstimatorTimerCallback, this));
    estimator_timer_->cancel();

    tracking_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / tracking_rate_),
        std::bind(&ObjectTrackingNode::TrackingTimerCallback, this));
    tracking_timer_->cancel();

    info_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / info_rate_),
        std::bind(&ObjectTrackingNode::TrackerInfoCallback, this));
    info_timer_->cancel();
}

void ObjectTrackingNode::CreatePublishers() {
    if (publish_fov_cloud_) {
        RCLCPP_WARN_ONCE(get_logger(),
                         "Field-of-view (FOV) cloud publishing enabled: this "
                         "is for debugging purposes only and will have a "
                         "negative effect on tracking performance.");
        fov_cloud_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud2>("points/fov", 1);
    }

    if (publish_roi_cloud_) {
        RCLCPP_WARN_ONCE(
            get_logger(),
            "Region of interest (ROI) cloud publishing enabled: this is for "
            "debugging purposes only and will have a negative effect on "
            "tracking performance.");
        roi_cloud_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud2>("points/roi", 1);
    }

    if (publish_ground_cloud_) {
        RCLCPP_WARN_ONCE(get_logger(),
                         "Ground cloud publishing enabled: this is for "
                         "debugging purposes only and will have a negative "
                         "effect on tracking performance.");
        ground_cloud_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud2>("points/ground", 1);
    }

    if (publish_cluster_cloud_) {
        RCLCPP_WARN_ONCE(
            get_logger(),
            "Cluster cloud publishing enabled: this is for debugging purposes "
            "only and will have a negative effect on tracking performance.");
        cluster_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "points/cluster", 1);
    }

    if (publish_cropbox_cloud_) {
        RCLCPP_WARN_ONCE(
            get_logger(),
            "Cropbox cloud publishing enabled: this is for debugging purposes "
            "only and will have a negative effect on tracking performance.");
        cropbox_cloud_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud2>("points/cropbox",
                                                            1);
    }

    detection_publisher_ =
        create_publisher<vision_msgs::msg::Detection3D>("detection_3d", 1);

    if (publish_image_) {
        image_publisher_ =
            create_publisher<sensor_msgs::msg::Image>("out_image", 1);
    }

    if (publish_pose_) {
        pose_publisher_ =
            create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "pose/raw", 1);

        pose_filtered_publisher_ =
            create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "pose/filtered", 1);
        }

    if (publish_odometry_) {
        odometry_publisher_ =
            create_publisher<nav_msgs::msg::Odometry>("odometry/raw", 1);

        odometry_filtered_publisher_ =
            create_publisher<nav_msgs::msg::Odometry>("odometry/filtered", 1);
    }

    info_publisher_ =
        create_publisher<avt_341_msgs::msg::TrackerInfo>("info", 1);
}

void ObjectTrackingNode::TrackerInfoCallback() {
    avt_341_msgs::msg::TrackerInfo info_message;
    info_message.header.stamp = get_clock()->now();
    info_message.header.frame_id = "none";
    info_message.state = state_;
    info_publisher_->publish(info_message);
}

void ObjectTrackingNode::CropRegionOfInterest() {
    RCLCPP_DEBUG(get_logger(),
                "Running crop box filter around the region of interest ...");

    // Define the crop box origin and its bounds relative to the last known
    // object centroid position. Note that we MUST use 4D floating-point vectors
    // to easily propagate through the affine transforms in the PCL library.
    crop_box_.setTranslation(bounding_box_centroid_.cast<float>());
    Eigen::Vector4f crop_box_roi_min(
        -roi_scale_factor_ * object_size_.x() / 2.0,
        -roi_scale_factor_ * object_size_.y() / 2.0,
        -roi_scale_factor_ * object_size_.z() / 2.0, 1.0f);
    Eigen::Vector4f crop_box_roi_max(roi_scale_factor_ * object_size_.x() / 2.0,
                                     roi_scale_factor_ * object_size_.y() / 2.0,
                                     roi_scale_factor_ * object_size_.z() / 2.0,
                                     1.0f);
    crop_box_.setMin(crop_box_roi_min);
    crop_box_.setMax(crop_box_roi_max);

    // Run the crop box filter in-place.
    crop_box_.setInputCloud(point_cloud_);
    crop_box_.filter(*point_cloud_);

    if (publish_cropbox_cloud_) {
        PublishPointCloud(point_cloud_, point_cloud_message_->header.stamp,
                          camera_frame_, cropbox_cloud_publisher_);
    }
}

void ObjectTrackingNode::TrackingTimerCallback() {
    RCLCPP_DEBUG_ONCE(get_logger(), "Tracking timer callback triggered!");
    if (!has_camera_info_) {
        state_ = TrackerState::INACTIVE;
        RCLCPP_DEBUG(get_logger(),
                     "No camera info received, skipping tracking ...");
        return;
    }

    if (!has_first_detection_) {
        state_ = TrackerState::INACTIVE;
        RCLCPP_DEBUG(get_logger(),
                     "No valid initial target detection received, skipping "
                     "tracking ...");
        return;
    }

    if (has_tracked_target_ && !has_detection_) {
        state_ = TrackerState::LIDAR_ONLY_TRACKING;
        RCLCPP_DEBUG(get_logger(),
                     "No camera target detection available, falling back to "
                     "LiDAR-only tracking ...");
    } else {
        state_ = TrackerState::FULL_TRACKING;
        RCLCPP_DEBUG(get_logger(), "Setting tracker to full tracking ...");
    }

    // Note: point cloud availability is no longer required. The tracking
    // measurement comes from the obstacle detector MarkerArray, not from
    // the internal PCL pipeline.

    if (sync_messages_) {
        // When sync is enabled use callback time to detect stale detections.
        // The point-cloud-timestamp path is removed because the point cloud
        // is no longer part of the tracking measurement loop.
        if (use_callback_time_) {
            const double detection_skew =
                get_clock()->now().nanoseconds() / 1.0e9 -
                last_valid_detection_callback_time_.nanoseconds() / 1.0e9;
            if (detection_skew > max_detection_skew_) {
                if (state_ != TrackerState::LIDAR_ONLY_TRACKING) {
                    RCLCPP_WARN(get_logger(),
                                "Last detection is too old %.2lf s, switching "
                                "to LiDAR-only tracking ...",
                                detection_skew);
                    state_ = TrackerState::LIDAR_ONLY_TRACKING;
                }
                has_detection_ = false;
            }
        }
    }

    auto start_time = get_clock()->now();

    // Tracking measurement via obstacle detector bounding box markers.
    // The LiDAR obstacle detector node runs its own clustering and tracking
    // pipeline and publishes bounding boxes as a MarkerArray. We subscribe
    // to that topic and use the marker positions as LiDAR measurements,
    // replacing the internal PCL clustering pipeline.

    if (state_ == TrackerState::LIDAR_ONLY_TRACKING) {
        // In LIDAR_ONLY mode we already have an associated obstacle ID from
        // a previous FULL_TRACKING cycle. Find that marker in the latest
        // MarkerArray and use its position as the measurement.
        if (tracked_obstacle_id_ < 0 || !has_obstacle_markers_) {
            RCLCPP_WARN(get_logger(),
                        "LIDAR_ONLY: no tracked obstacle ID (%d) or no "
                        "obstacle markers received yet.",
                        tracked_obstacle_id_);
            has_point_cloud_ = false;
            has_detection_ = false;
            CheckTargetTimeout();
            return;
        }

        bool found = false;
        for (const auto& marker : latest_obstacle_markers_.markers) {
            if (marker.action ==
                visualization_msgs::msg::Marker::DELETEALL) {
                continue;
            }
            if (marker.id != tracked_obstacle_id_) {
                continue;
            }

            // Transform marker position from its native frame to camera frame
            // so that the EstimatorTimerCallback can handle it uniformly.
            const Eigen::Vector3d marker_pos(marker.pose.position.x,
                                             marker.pose.position.y,
                                             marker.pose.position.z);
            bounding_box_centroid_ = TransformToCoordinates(
                marker.header.frame_id, camera_frame_, marker_pos);
            centroid_in_cloud_frame_ = false;

            bounding_box_size_ = Eigen::Vector3d(
                marker.scale.x, marker.scale.y, marker.scale.z);
            object_size_ = bounding_box_size_;
            bounding_box_orientation_ = Eigen::Quaterniond(
                marker.pose.orientation.w, marker.pose.orientation.x,
                marker.pose.orientation.y, marker.pose.orientation.z);

            has_new_measurement_ = true;
            has_had_first_lidar_measurement_ = true;
            has_tracked_target_ = true;
            last_valid_target_time_ = get_clock()->now();
            last_lidar_seen_time_ = last_valid_target_time_;
            // Record world-frame position for re-acquisition after a brief
            // drop-out: if the obstacle disappears and reappears with a new
            // ID within lidar_reacquire_max_time_, we match it by proximity.
            last_lidar_world_pos_ = TransformToCoordinates(
                marker.header.frame_id, world_frame_, marker_pos);
            found = true;

            RCLCPP_INFO(get_logger(),
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
            // since the obstacle was last seen is within lidar_reacquire_max_time_,
            // search all current markers for one whose world-frame position is
            // within lidar_reacquire_max_dist_ of the last known position.
            // If found, adopt its ID — the LiDAR briefly lost the obstacle and
            // reassigned a new ID when it reappeared.
            const double elapsed_since_seen =
                (get_clock()->now() - last_lidar_seen_time_).seconds();

            if (elapsed_since_seen < lidar_reacquire_max_time_) {
                int reacquire_id = -1;
                double best_dist = lidar_reacquire_max_dist_;
                Eigen::Vector3d best_cam_pos;
                Eigen::Vector3d best_size;
                Eigen::Quaterniond best_quat = Eigen::Quaterniond::Identity();

                for (const auto& m : latest_obstacle_markers_.markers) {
                    if (m.action == visualization_msgs::msg::Marker::DELETEALL)
                        continue;
                    if (m.id == tracked_obstacle_id_)
                        continue;  // already checked above, not present
                    const Eigen::Vector3d mpos(m.pose.position.x,
                                               m.pose.position.y,
                                               m.pose.position.z);
                    const Eigen::Vector3d mpos_world = TransformToCoordinates(
                        m.header.frame_id, world_frame_, mpos);
                    const double d = (mpos_world - last_lidar_world_pos_).norm();
                    if (d < best_dist) {
                        best_dist = d;
                        reacquire_id = m.id;
                        best_cam_pos = TransformToCoordinates(
                            m.header.frame_id, camera_frame_, mpos);
                        best_size = Eigen::Vector3d(
                            m.scale.x, m.scale.y, m.scale.z);
                        best_quat = Eigen::Quaterniond(
                            m.pose.orientation.w, m.pose.orientation.x,
                            m.pose.orientation.y, m.pose.orientation.z);
                    }
                }

                if (reacquire_id >= 0) {
                    RCLCPP_INFO(get_logger(),
                                "LIDAR_ONLY: re-acquired obstacle as new ID %d "
                                "(was %d, %.2f m away, %.2f s gap).",
                                reacquire_id, tracked_obstacle_id_,
                                best_dist, elapsed_since_seen);
                    tracked_obstacle_id_ = reacquire_id;
                    bounding_box_centroid_ = best_cam_pos;
                    centroid_in_cloud_frame_ = false;
                    bounding_box_size_ = best_size;
                    object_size_ = best_size;
                    bounding_box_orientation_ = best_quat;
                    has_new_measurement_ = true;
                    has_had_first_lidar_measurement_ = true;
                    has_tracked_target_ = true;
                    last_valid_target_time_ = get_clock()->now();
                    last_lidar_seen_time_ = last_valid_target_time_;
                    last_lidar_world_pos_ = TransformToCoordinates(
                        camera_frame_, world_frame_, best_cam_pos);
                    // Skip further processing — measurement is ready.
                } else {
                    RCLCPP_WARN(get_logger(),
                                "LIDAR_ONLY: obstacle ID %d not present in latest "
                                "markers; waiting for it to reappear (%.2f s elapsed).",
                                tracked_obstacle_id_, elapsed_since_seen);
                    has_point_cloud_ = false;
                    has_detection_ = false;
                    CheckTargetTimeout();
                    return;
                }
            } else {
                RCLCPP_WARN(get_logger(),
                            "LIDAR_ONLY: obstacle ID %d not present in latest "
                            "markers; waiting for it to reappear.",
                            tracked_obstacle_id_);
                has_point_cloud_ = false;
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
        if (!has_obstacle_markers_) {
            RCLCPP_WARN(get_logger(),
                        "FULL_TRACKING: no obstacle markers received yet, "
                        "falling back to camera-only tracking.");
            CameraCentroidEstimate();
            state_ = TrackerState::CAMERA_ONLY_TRACKING;
            has_point_cloud_ = false;
            has_detection_ = true;
            has_tracked_target_ = true;
            last_valid_target_time_ = get_clock()->now();
            CheckTargetTimeout();
            return;
        }

        // Detection bounding-box center in pixel coordinates.
        const double det_u =
            detections_message_.bbox.center.position.x;
        const double det_v =
            detections_message_.bbox.center.position.y;

        // Maximum allowed pixel distance: the half-diagonal of the detection
        // bounding box, scaled by obstacle_association_max_dist_. Using
        // obstacle_association_max_dist_ = 1.0 means the marker's projection
        // must land within one half-diagonal of the bbox center. Increase it
        // to be more permissive.
        const double bbox_half_diag = 0.5 * std::sqrt(
            detections_message_.bbox.size_x *
            detections_message_.bbox.size_x +
            detections_message_.bbox.size_y *
            detections_message_.bbox.size_y);
        const double max_pixel_dist =
            obstacle_association_max_dist_ * bbox_half_diag;

        double best_pixel_dist = max_pixel_dist;
        int best_id = -1;
        Eigen::Vector3d best_pos_cam;
        Eigen::Vector3d best_size;
        Eigen::Quaterniond best_quat = Eigen::Quaterniond::Identity();

        for (const auto& marker : latest_obstacle_markers_.markers) {
            if (marker.action ==
                visualization_msgs::msg::Marker::DELETEALL) {
                continue;
            }

            // Transform marker position to camera frame.
            const Eigen::Vector3d pos(marker.pose.position.x,
                                      marker.pose.position.y,
                                      marker.pose.position.z);
            const Eigen::Vector3d pos_cam = TransformToCoordinates(
                marker.header.frame_id, camera_frame_, pos);

            // Skip markers behind the camera.
            if (pos_cam.z() <= 0.0) continue;

            // Project to image pixel coordinates using camera intrinsics.
            const double fx = camera_info_message_->k[0];
            const double fy = camera_info_message_->k[4];
            const double cx = camera_info_message_->k[2];
            const double cy = camera_info_message_->k[5];
            const double proj_u = fx * pos_cam.x() / pos_cam.z() + cx;
            const double proj_v = fy * pos_cam.y() / pos_cam.z() + cy;

            // Skip projections outside the image.
            if (proj_u < 0.0 ||
                proj_u > static_cast<double>(camera_info_message_->width) ||
                proj_v < 0.0 ||
                proj_v > static_cast<double>(camera_info_message_->height)) {
                continue;
            }

            const double pixel_dist = std::sqrt(
                (proj_u - det_u) * (proj_u - det_u) +
                (proj_v - det_v) * (proj_v - det_v));

            if (pixel_dist < best_pixel_dist) {
                best_pixel_dist = pixel_dist;
                best_id = marker.id;
                best_pos_cam = pos_cam;
                best_size = Eigen::Vector3d(
                    marker.scale.x, marker.scale.y, marker.scale.z);
                best_quat = Eigen::Quaterniond(
                    marker.pose.orientation.w, marker.pose.orientation.x,
                    marker.pose.orientation.y, marker.pose.orientation.z);
            }
        }

        if (best_id < 0) {
            // No obstacle marker projects near the camera detection bbox.
            // Fall back to the camera range estimate until the obstacle
            // detector picks up the target again.
            RCLCPP_INFO(get_logger(),
                        "FULL_TRACKING: no obstacle marker projects within "
                        "%.0f px of detection center (%.0f, %.0f); "
                        "using camera-only measurement.",
                        max_pixel_dist, det_u, det_v);
            CameraCentroidEstimate();
            state_ = TrackerState::CAMERA_ONLY_TRACKING;
            has_point_cloud_ = false;
            has_detection_ = true;
            has_tracked_target_ = true;
            last_valid_target_time_ = get_clock()->now();
            CheckTargetTimeout();
            return;
        }

        // Associate this obstacle with the target.
        tracked_obstacle_id_ = best_id;
        bounding_box_centroid_ = best_pos_cam;
        centroid_in_cloud_frame_ = false;
        bounding_box_size_ = best_size;
        object_size_ = best_size;
        bounding_box_orientation_ = best_quat;

        // On the first LiDAR lock-on, reset the Kalman filter so it
        // re-initializes from the reliable LiDAR position rather than
        // continuing from the noisy camera-only state that preceded it.
        // Subsequent LiDAR updates carry on from the already-initialized filter.
        if (!has_had_first_lidar_measurement_) {
            RCLCPP_INFO(get_logger(),
                        "FULL_TRACKING: first LiDAR lock on obstacle ID %d; "
                        "re-initializing filter from LiDAR position.",
                        best_id);
            filter_initialized_ = false;
        }

        has_new_measurement_ = true;
        has_had_first_lidar_measurement_ = true;
        has_tracked_target_ = true;
        last_valid_target_time_ = get_clock()->now();
        last_lidar_seen_time_ = last_valid_target_time_;
        last_lidar_world_pos_ = TransformToCoordinates(
            camera_frame_, world_frame_, best_pos_cam);

        RCLCPP_INFO(get_logger(),
                    "FULL_TRACKING: associated obstacle ID %d "
                    "(projected %.0f px from detection center).",
                    tracked_obstacle_id_, best_pixel_dist);
    }

    if (publish_image_) {
        PublishImage();
    }

    execution_time_ = (get_clock()->now() - start_time).nanoseconds() / 1.0e6;
    RCLCPP_DEBUG(get_logger(), "Tracker pipeline execution time: %0.2lf ms",
                 execution_time_);

    has_point_cloud_ = false;
    has_detection_ = false;

    // Check for tracking timeout across all tracking modes (FULL_TRACKING,
    // LIDAR_ONLY_TRACKING, and CAMERA_ONLY_TRACKING). This ensures that if
    // no valid measurement is obtained for longer than target_timeout_, the
    // tracker transitions to NO_DETECTION state.
    CheckTargetTimeout();
}

void ObjectTrackingNode::CheckTargetTimeout() {
    // Timeout fires only when neither camera nor LiDAR has produced a valid
    // measurement for longer than target_timeout_. last_valid_target_time_ is
    // updated by any valid measurement from either source, so this correctly
    // keeps the tracker alive as long as at least one sensor sees the target.
    if ((get_clock()->now() - last_valid_target_time_).seconds() >
        target_timeout_) {
        RCLCPP_WARN(get_logger(), "Tracker timeout reached.");
        state_ = TrackerState::NO_DETECTION;
        has_tracked_target_ = false;
    }
}

void ObjectTrackingNode::EuclideanClustering() {
    try {
        // In LIDAR_ONLY mode, pass the predicted target position as the
        // reference point so the best cluster is the one nearest to where
        // the target is expected to be, not the one nearest to the sensor.
        Eigen::Vector3f reference = Eigen::Vector3f::Zero();
        if (state_ == TrackerState::LIDAR_ONLY_TRACKING) {
            reference = bounding_box_centroid_.cast<float>();
        }
        auto clustering_result = ExtractEuclideanClusters(point_cloud_, reference);
        // Update the flags for centroid measurement.
        has_new_measurement_ = true;
        has_had_first_lidar_measurement_ = true;

        cloud_cluster_ = clustering_result.first;
        bounding_box_centroid_ = Eigen::Vector3d(clustering_result.second.x,
                                                 clustering_result.second.y,
                                                 clustering_result.second.z);

        if (publish_cluster_cloud_) {
            PublishPointCloud(cloud_cluster_,
                              point_cloud_message_->header.stamp, camera_frame_,
                              cluster_publisher_);
        }
    } catch (const ClusteringException& exception) {
        RCLCPP_DEBUG_STREAM(get_logger(),
                            "Clustering exception: " << exception.what());
        throw;
    }
}

void ObjectTrackingNode::Reset() {
    execution_time_ = -1.0;
}

void ObjectTrackingNode::ObstacleMarkersCallback(
    const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    // Always store the latest message, including DELETEALL-only messages.
    // A DELETEALL-only message signals that the obstacle detector found no
    // obstacles this frame; ignoring it would leave latest_obstacle_markers_
    // stale with the previous frame's boxes, preventing tracking timeout.
    latest_obstacle_markers_ = *msg;
    has_obstacle_markers_ = true;
}

// JN addition for camera detection only tracking
void ObjectTrackingNode::CameraCentroidEstimate() {
    try {
        // bbox already in camera before update
        Eigen::Vector3d camera_centroid_rdf =
            ObjectTrackingNode::ConvertBBoxCoordinatesToPoseCentroid_rdf(
                detections_message_,
                camera_info_message_);
        // Swap to flu
        bounding_box_centroid_ = Eigen::Vector3d(camera_centroid_rdf.x(),
                                                 camera_centroid_rdf.y(),
                                                 camera_centroid_rdf.z());
        // bbox already in camera before update
        //bounding_box_centroid_ = TransformToCoordinates(
        //    detections_message_.header.frame_id,
        //    point_cloud_message_->header.frame_id,
        //    camera_centroid_rdf);
        has_new_measurement_ = true;
    } catch (...) {
        RCLCPP_INFO_STREAM(get_logger(),
        "Camera centroid failed: ");
        has_new_measurement_ = false;
        throw;
    }
}

void ObjectTrackingNode::TransformPointCloudToCameraFrame(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message) {
    if (point_cloud_message->header.frame_id.empty() ||
        !(transform_buffer_->canTransform(camera_frame_,
                                          point_cloud_message->header.frame_id,
                                          rclcpp::Time(0)))) {

        std::string message(
            "Could not lookup the transform between point cloud frame "
            "and camera frame!");
        RCLCPP_ERROR(get_logger(), message.c_str());
        throw TransformException(message.c_str());
    }

    if (tf2::getFrameId(*point_cloud_message) != camera_frame_) {
        RCLCPP_DEBUG(get_logger(),
                     "Transforming point cloud between LiDAR frame \"%s\" and "
                     "camera frame \"%s\" ...",
                     tf2::getFrameId(*point_cloud_message).c_str(),
                     camera_frame_.c_str());
        try {
            auto transform_message =
                TransformPointCloud(point_cloud_message, camera_frame_);

            pcl::transformPointCloud(
                *point_cloud, *point_cloud,
                tf2::transformToEigen(transform_message.transform).matrix());
        } catch (tf2::TransformException& exception) {
            RCLCPP_WARN(get_logger(), exception.what());
            return;
        }
    }
}

Eigen::Vector3d ObjectTrackingNode::TransformToCoordinates(
    const std::string& source_frame, const std::string& target_frame,
    const Eigen::Vector3d& point) const {
    geometry_msgs::msg::TransformStamped transform_message;
    try {
        transform_message = transform_buffer_->lookupTransform(
            target_frame.c_str(), source_frame.c_str(), tf2::TimePointZero);
    } catch (tf2::TransformException& exception) {
        RCLCPP_ERROR(get_logger(), "Transform lookup exception.");
        return point;
    }

    Eigen::Vector3d transformed_point;
    tf2::doTransform(point, transformed_point, transform_message);
    return transformed_point;
}

void ObjectTrackingNode::PointCloudCallback(
    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message) {
    RCLCPP_DEBUG_ONCE(get_logger(), "Point cloud callback triggered!");

    // Convert the ROS sensor_msgs/msg/PointCloud2 message to a PCL XYZ
    // point cloud.
    point_cloud_ = ToPCLCloud(point_cloud_message);
    point_cloud_message_ = point_cloud_message;
    has_point_cloud_ = true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ObjectTrackingNode::ToPCLCloud(
    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*point_cloud_message, *point_cloud);

    RCLCPP_DEBUG(get_logger(), "Number of points in the raw point cloud: %i",
                 int(point_cloud->points.size()));

    return point_cloud;
}

void ObjectTrackingNode::RemoveNaNPoints(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {
    RCLCPP_DEBUG(get_logger(),
                 "Removing invalid values (NaN) from the point cloud ...");

    unsigned int points_before_filtering = point_cloud->points.size();

    std::vector<int> removed_indices;
    pcl::removeNaNFromPointCloud(*point_cloud, *point_cloud, removed_indices);

    unsigned int points_after_filtering = point_cloud->points.size();

    RCLCPP_DEBUG(get_logger(),
                 "Total points after NaN filtering: %i -> %i (%i)",
                 points_before_filtering, points_after_filtering,
                 points_before_filtering - points_after_filtering);
}

void ObjectTrackingNode::ImageCallback(
    const sensor_msgs::msg::Image::SharedPtr image_message) {
    RCLCPP_DEBUG_ONCE(get_logger(), "Camera image callback triggered!");

    try {
        // Store the sensor_msgs/msg/Image as an OpenCV image and mark the
        // camera image as received.
        latest_image_ = cv_bridge::toCvShare(image_message);
        has_image_ = true;
    } catch (const cv_bridge::Exception& exception) {
        RCLCPP_ERROR(get_logger(), "CVBridge exception: %s", exception.what());
    }
}

void ObjectTrackingNode::CameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_message) {
    RCLCPP_DEBUG_ONCE(get_logger(), "Camera info callback triggered!!");

    // Store the sensor_msgs/msg/CameraInfo message and mark camera info as
    // received.
    camera_info_message_ = camera_info_message;
    has_camera_info_ = true;
}

void ObjectTrackingNode::DetectionsCallback(
    const vision_msgs::msg::Detection2DArray::SharedPtr detections_message) {
    RCLCPP_DEBUG_ONCE(get_logger(), "Detections callback triggered!");

    if (!has_target_selection_) {
        RCLCPP_INFO(get_logger(),
                    "No target selected, ignoring detections ...");
        has_detection_ = false;
        return;
    }

    if (detections_message->detections.empty()) {
        RCLCPP_DEBUG(get_logger(),
                     "No detections in the current frame, skipping ...");
        has_detection_ = false;
        return;
    }

    int target_idx = -1;
    for (size_t i = 0; i < detections_message->detections.size(); ++i) {
        // We only consider the highest scoring result for each detection,
        // under the assumption that the hypotheses array is sorted from
        // highest to lowest scoring.
        const auto & detection = detections_message->detections[i];
        if (detection.results[0].hypothesis.class_id == target_class_) {
            detection_score_ = detection.results[0].hypothesis.score;
            target_idx = static_cast<int>(i);
            break;
        }
    }

    if (target_idx == -1) {
        RCLCPP_INFO(get_logger(),
                    "Target %s not found in the current detection, skipping ...", target_class_.c_str());
        has_detection_ = false;
        return;
    }

    // Reject the detection if the bounding box touches any image edge.
    // A clipped bbox produces a biased centroid estimate because part of the
    // object is outside the frame; the range/bearing estimate from
    // ConvertBBoxCoordinatesToPoseCentroid_rdf becomes unreliable.
    if (has_camera_info_) {
        const auto& bbox = detections_message->detections[target_idx].bbox;
        const double left   = bbox.center.position.x - bbox.size_x / 2.0;
        const double right  = bbox.center.position.x + bbox.size_x / 2.0;
        const double top    = bbox.center.position.y - bbox.size_y / 2.0;
        const double bottom = bbox.center.position.y + bbox.size_y / 2.0;
        if (left <= 0.0 ||
            right  >= static_cast<double>(camera_info_message_->width) ||
            top    <= 0.0 ||
            bottom >= static_cast<double>(camera_info_message_->height)) {
            RCLCPP_DEBUG(get_logger(),
                         "Bounding box touches image edge (l=%.1f r=%.1f "
                         "t=%.1f b=%.1f img=%ux%u) — skipping camera update.",
                         left, right, top, bottom,
                         camera_info_message_->width,
                         camera_info_message_->height);
            has_detection_ = false;
            return;
        }
    }

    // Store the vision_msgs/msg/Detection2D message, keep track of its
    // timestamp and mark detections as received.
    detections_message_ = detections_message->detections[target_idx];
    last_valid_detection_time_ = detections_message->header.stamp;
    last_valid_detection_callback_time_ = get_clock()->now();

    has_first_detection_ = true;
    has_detection_ = true;
}

void ObjectTrackingNode::DownsampleCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {
    RCLCPP_DEBUG(get_logger(),
                 "Applying voxel grid downsampling with leaf size %0.3f ...",
                 leaf_size_);

    // Filter the point cloud in-place.
    voxel_grid_filter_.setInputCloud(point_cloud);
    voxel_grid_filter_.filter(*point_cloud);

    RCLCPP_DEBUG(get_logger(),
                 "Number of points in the processed point cloud after voxel "
                 "grid downsampling: %i",
                 int(point_cloud->points.size()));
}

geometry_msgs::msg::TransformStamped ObjectTrackingNode::TransformPointCloud(
    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message,
    const std::string target_frame) {
    // Retrieve the transform from the point cloud message native frame to
    // the target frame from the published transform tree.
    try {
        return transform_buffer_->lookupTransform(
            target_frame, tf2::getFrameId(*point_cloud_message),
            tf2::TimePointZero);
    } catch (tf2::TransformException& exception) {
        RCLCPP_ERROR(get_logger(), "Transform lookup exception.");
        throw;
    }
}

// JN addition for camera detection only tracking
// use vehicle height to estimate centroid using range from boundingbox height
// expressed in righ-down-front (rdf) frame
// Added to code by Jonas N
Eigen::Vector3d ObjectTrackingNode::ConvertBBoxCoordinatesToPoseCentroid_rdf(
    const vision_msgs::msg::Detection2D& detections_message,
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_message) {
    const double car_size_z = camera_target_height_;
    double target_z_f = (double)camera_info_message->k[4] / (double)detections_message.bbox.size_y *
        car_size_z;
    double target_x_r = target_z_f / (double)camera_info_message->k[0] *
        (double)(detections_message.bbox.center.position.x - camera_info_message->k[2]);

    double target_y_d = target_z_f / (double)camera_info_message->k[4] *
        (double)(detections_message.bbox.center.position.y - camera_info_message->k[5]);
    Eigen::Vector3d camera_estimated_centroid_rdf(target_x_r, target_y_d, target_z_f);
	
	// covariance jacobians
	const double s2_pixel = camera_bbox_pixel_sigma_ * camera_bbox_pixel_sigma_;
	double s2_forwards = (double)camera_info_message->k[4] / pow((double)detections_message.bbox.size_y,2) *
        car_size_z * s2_pixel * (double)camera_info_message->k[4] /
			pow((double)detections_message.bbox.size_y, 2) *
        car_size_z;
	double s2_right = target_z_f / (double)camera_info_message->k[0] * s2_pixel *
		target_z_f / (double)camera_info_message->k[0];
	double s2_down = target_z_f / (double)camera_info_message->k[4] * s2_pixel *
		target_z_f / (double)camera_info_message->k[4];

	R_rdf_(0, 0) = std::max(filter_measurement_variance_,s2_right);
	R_rdf_(1, 1) = std::max(filter_measurement_variance_, s2_down);
	R_rdf_(2, 2) = std::max(filter_measurement_variance_, s2_forwards);
    RCLCPP_INFO_STREAM(get_logger(), "ConvertBBoxCoordinatesToPoseCentroid of size " << detections_message.bbox.size_y << " pixel, " << car_size_z << "m" << '\n'
        << "[x, y ,z] = [ " << target_x_r
        << ", " << target_y_d
        << ", " << target_z_f << "]" << '\n');
    return camera_estimated_centroid_rdf;
}

PixelCoordinates ObjectTrackingNode::ConvertPointToPixelCoordinates(
    const pcl::PointXYZ& point,
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_message) {
    return PixelCoordinates(camera_info_message->k[0] * point.x / point.z +
                                camera_info_message->k[2],
                            camera_info_message->k[4] * point.y / point.z +
                                camera_info_message->k[5]);
}

std::vector<PixelCoordinates>
ObjectTrackingNode::ConvertPointCloudToPixelCoordinates(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_message) {
    std::vector<PixelCoordinates> coordinates;
    coordinates.reserve(point_cloud->size());

    for (auto& point : point_cloud->points) {
        coordinates.emplace_back(
            ConvertPointToPixelCoordinates(point, camera_info_message));
    }

    return coordinates;
}

void ObjectTrackingNode::FindPointsInCameraFOV(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
    const std::vector<PixelCoordinates>& coordinates, const int height,
    const int width) {
    pcl::PointIndices::Ptr fov_points_indices(new pcl::PointIndices());
    fov_points_indices->indices.reserve(point_cloud->size());

    for (unsigned int index = 0; index < coordinates.size(); ++index) {
        if (point_cloud->points[index].z > 0.0 && coordinates[index].x_ >= 0 &&
            coordinates[index].x_ <= width && coordinates[index].y_ >= 0 &&
            coordinates[index].y_ <= height) {
            fov_points_indices->indices.push_back(index);
        }
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract_fov_indices;
    extract_fov_indices.setInputCloud(point_cloud);
    extract_fov_indices.setIndices(fov_points_indices);
    extract_fov_indices.setNegative(false);
    extract_fov_indices.setKeepOrganized(true);
    extract_fov_indices.filterDirectly(point_cloud);
}

void ObjectTrackingNode::FindPointsInROI(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
    const std::vector<PixelCoordinates>& coordinates, const unsigned int x_min,
    const unsigned int x_max, const unsigned int y_min,
    const unsigned int y_max) {
    pcl::PointIndices::Ptr roi_points_indices(new pcl::PointIndices());

    for (unsigned int index = 0; index < coordinates.size(); ++index) {
        if (coordinates[index].x_ >= x_min && coordinates[index].x_ < x_max &&
            coordinates[index].y_ >= y_min && coordinates[index].y_ < y_max) {
            roi_points_indices->indices.push_back(index);
        }
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract_fov_indices;
    extract_fov_indices.setInputCloud(point_cloud);
    extract_fov_indices.setIndices(roi_points_indices);
    extract_fov_indices.setNegative(false);
    extract_fov_indices.setKeepOrganized(true);
    extract_fov_indices.filterDirectly(point_cloud);
}

void ObjectTrackingNode::LimitSensorDistance(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, bool symmetric) {
    passthrough_filter_.setInputCloud(point_cloud);
    if (symmetric) {
        passthrough_filter_.setFilterLimits(-passthrough_distance_max_,
                                            passthrough_distance_max_);
        passthrough_filter_.filter(*point_cloud);
        passthrough_filter_.setNegative(true);
        passthrough_filter_.setFilterLimits(-passthrough_distance_min_,
                                            passthrough_distance_min_);
        passthrough_filter_.filter(*point_cloud);
        passthrough_filter_.setNegative(false);
    } else {
        passthrough_filter_.setFilterLimits(passthrough_distance_min_,
                                            passthrough_distance_max_);
        passthrough_filter_.filter(*point_cloud);
    }
}

const std::pair<const pcl::PointCloud<pcl::PointXYZ>::Ptr, const pcl::PointXYZ>
ObjectTrackingNode::ExtractEuclideanClusters(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
    const Eigen::Vector3f& reference_point) {
    // Instantiate a k-d tree to speed up the clustering process.
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    kd_tree->setInputCloud(point_cloud);

    // Configure the Euclidean cluster extraction agent.
    // Use a conservative floor based on cluster_size_min_ to reduce wasted
    // processing on undersized clusters. The distance-scaled minimum is
    // enforced per cluster after extraction for more precise validation.
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_clustering_;
    euclidean_clustering_.setClusterTolerance(clustering_tolerance_);
    euclidean_clustering_.setMinClusterSize(std::max(3, static_cast<int>(cluster_size_min_)));
    euclidean_clustering_.setMaxClusterSize(cluster_size_max_);
    euclidean_clustering_.setSearchMethod(kd_tree);

    // Run the cluster extraction algorithm and store the results in a
    // vector of vectors of cluster indices.
    euclidean_clustering_.setInputCloud(point_cloud);
    std::vector<pcl::PointIndices> clusters_indices;
    euclidean_clustering_.extract(clusters_indices);

    // Throw an exception to be caught upstream if clustering was unsuccessful.
    if (uint(clusters_indices.size()) < 1) {
        throw ClusteringException(
            "Could not find any valid clusters in the provided point "
            "cloud.");
    }

    RCLCPP_DEBUG(get_logger(), "Euclidean clustering found %i clusters.",
                 uint(clusters_indices.size()));

    // Create a shared pointer to a new point cloud to store the closest
    // cluster. When a non-zero reference point is provided (LIDAR_ONLY mode),
    // "closest" means closest to the predicted target position; otherwise it
    // means closest to the sensor origin (FULL_TRACKING default).
    pcl::PointCloud<pcl::PointXYZ>::Ptr closest_cloud_cluster(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ closest_cluster_centroid(
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity());
    float closest_distance = std::numeric_limits<float>::infinity();

    // Iterate over the cloud clusters to compute the cluster centroids.
    bool found_valid_cluster = false;
    for (unsigned int cluster_index = 0;
         cluster_index < uint(clusters_indices.size()); ++cluster_index) {
        // Temporarily instantiate a point cloud to the current cluster.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
            new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : clusters_indices[cluster_index].indices) {
            cloud_cluster->push_back((*point_cloud)[idx]);
        }

        pcl::PointXYZ cloud_centroid;
        pcl::computeCentroid(*cloud_cluster, cloud_centroid);

        // Bounding-box and point-count sanity checks applied uniformly in
        // all tracking modes. In LIDAR_ONLY mode, ground plane removal is
        // skipped upstream so cluster extents are not artificially clipped,
        // and these thresholds apply cleanly to the full cluster geometry.

        // Minimum point count.
        // In LIDAR_ONLY mode, cluster_size_min_ is used as a fixed absolute
        // floor. The dynamic 1/d^2 formula is not applied because the crop
        // box puts the target close to the sensor origin, making the formula
        // produce an unreasonably large threshold.
        // In FULL_TRACKING the threshold is distance-scaled because the cloud
        // covers the full scene and long-range targets naturally return fewer
        // points.
        {
            const int min_points =
                (state_ == TrackerState::LIDAR_ONLY_TRACKING)
                    ? cluster_size_min_
                    : std::max(3, static_cast<int>(
                          cluster_size_min_ *
                          std::pow(cluster_distance_ref_ /
                              std::max(cloud_centroid.getVector3fMap().norm(),
                                       0.1f), 2)));
            if (static_cast<int>(cloud_cluster->points.size()) < min_points) {
                RCLCPP_WARN(
                    get_logger(),
                    "Cluster %i rejected: %i points < minimum %i.",
                    cluster_index,
                    static_cast<int>(cloud_cluster->points.size()),
                    min_points);
                continue;
            }
        }

        // Maximum point count. This should already be enforced by Euclidean
        // clustering, but we validate explicitly here for safety and clarity.
        if (static_cast<int>(cloud_cluster->points.size()) > cluster_size_max_) {
            RCLCPP_WARN(
                get_logger(),
                "Cluster %i rejected: %i points > maximum %i.",
                cluster_index,
                static_cast<int>(cloud_cluster->points.size()),
                cluster_size_max_);
            continue;
        }

        // Bounding-box dimension checks. In the camera optical frame:
        //   X = left-right (width), Y = down (height), Z = forward (depth).
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);
        const float cluster_height = max_pt.y - min_pt.y;
        const float cluster_width  = max_pt.x - min_pt.x;
        const float cluster_depth  = max_pt.z - min_pt.z;

        if (cluster_height < static_cast<float>(cluster_height_min_) ||
            cluster_height > static_cast<float>(cluster_height_max_)) {
            RCLCPP_WARN(
                get_logger(),
                "Cluster %i rejected: height %.2f m outside [%.2f, %.2f] m.",
                cluster_index, cluster_height,
                cluster_height_min_, cluster_height_max_);
            continue;
        }
        if (cluster_width < static_cast<float>(cluster_width_min_) ||
            cluster_width > static_cast<float>(cluster_width_max_)) {
            RCLCPP_WARN(
                get_logger(),
                "Cluster %i rejected: width %.2f m outside [%.2f, %.2f] m.",
                cluster_index, cluster_width,
                cluster_width_min_, cluster_width_max_);
            continue;
        }
        if (cluster_depth < static_cast<float>(cluster_depth_min_) ||
            cluster_depth > static_cast<float>(cluster_depth_max_)) {
            RCLCPP_WARN(
                get_logger(),
                "Cluster %i rejected: depth %.2f m outside [%.2f, %.2f] m.",
                cluster_index, cluster_depth,
                cluster_depth_min_, cluster_depth_max_);
            continue;
        }

        found_valid_cluster = true;

        // Select the cluster closest to the reference point. In LIDAR_ONLY
        // mode this picks the cluster nearest the predicted target position.
        // In FULL_TRACKING mode (reference = zero) it picks the cluster
        // nearest the sensor origin.
        const float selection_distance =
            (cloud_centroid.getVector3fMap() - reference_point).norm();
        if (selection_distance < closest_distance) {
            closest_distance = selection_distance;
            closest_cluster_centroid = cloud_centroid;
            closest_cloud_cluster = cloud_cluster;
        }

        RCLCPP_DEBUG(
            get_logger(), "Cluster %i: %i points, %.2f m from reference.",
            cluster_index, uint(cloud_cluster->points.size()), selection_distance);
    }

    if (!found_valid_cluster) {
        throw ClusteringException(
            "All clusters failed sanity checks (point count or height).");
    }

    return std::pair<const pcl::PointCloud<pcl::PointXYZ>::Ptr,
                     const pcl::PointXYZ>(closest_cloud_cluster,
                                          closest_cluster_centroid);
}

void ObjectTrackingNode::SegmentGroundPlane(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane) {
    // Derive the ground plane normal from vehicle attitude via TF. The
    // transform from world to camera frame reflects the IMU-estimated
    // orientation of the vehicle, so the world "up" direction rotated into
    // camera frame gives the true ground normal even when the vehicle is
    // turning or traversing rough terrain where the plane is no longer
    // near-horizontal in the sensor frame.
    try {
        auto tf_msg = transform_buffer_->lookupTransform(
            camera_frame_, world_frame_, tf2::TimePointZero);
        Eigen::Quaternionf q(
            static_cast<float>(tf_msg.transform.rotation.w),
            static_cast<float>(tf_msg.transform.rotation.x),
            static_cast<float>(tf_msg.transform.rotation.y),
            static_cast<float>(tf_msg.transform.rotation.z));
        // In ROS convention, world Z is up; rotating it into camera frame
        // gives the ground plane normal in local coordinates.
        Eigen::Vector3f ground_normal =
            (q * Eigen::Vector3f(0.0f, 0.0f, 1.0f)).normalized();
        sac_segmentation_.setAxis(ground_normal);
    } catch (const tf2::TransformException&) {
        // TF not yet available; keep the static axis set during initialization.
    }

    // Segment the largest planar component from the remaining cloud
    sac_segmentation_.setInputCloud(point_cloud);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    sac_segmentation_.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        RCLCPP_DEBUG(get_logger(),
                     "Could not estimate a planar model for the given "
                     "dataset.");
        return;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setInputCloud(point_cloud);
    extract_indices.setIndices(inliers);
    extract_indices.setNegative(false);
    extract_indices.setKeepOrganized(true);

    // Get the points associated with the planar surface
    extract_indices.filter(*cloud_plane);

    RCLCPP_DEBUG(
        get_logger(),
        "Point cloud representing the planar component: %i data points.",
        int(cloud_plane->size()));

    // Remove the planar inliers, extract the rest
    extract_indices.setNegative(true);
    extract_indices.setKeepOrganized(true);
    extract_indices.filterDirectly(point_cloud);

    RemoveNaNPoints(point_cloud);

    if (publish_ground_cloud_) {
        PublishPointCloud(cloud_plane, point_cloud_message_->header.stamp,
                          camera_frame_, ground_cloud_publisher_);
    }
}

void ObjectTrackingNode::EstimatorTimerCallback() {
    RCLCPP_DEBUG_ONCE(get_logger(), "Estimator timer callback triggered!");

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
    if ((get_clock()->now() - timeout_reference).seconds() >
        target_timeout_) {
        if (state_ == TrackerState::LIDAR_ONLY_TRACKING ||
            state_ == TrackerState::NO_DETECTION) {
            RCLCPP_WARN(get_logger(),
                        "Target timeout, resetting estimator ...");
            filter_->SetInitialPosition(Eigen::Vector<double, 3>::Zero());
            filter_->SetInitialVelocity(Eigen::Vector<double, 3>::Zero());
            state_ = TrackerState::INACTIVE;
            has_detection_ = false;
            has_first_detection_ = false;
            filter_initialized_ = false;
            has_had_first_lidar_measurement_ = false;
            tracked_obstacle_id_ = -1;
        }
    }

    if (!filter_initialized_) {
        filter_->SetInitialPosition(TransformToCoordinates(
            camera_frame_, world_frame_, bounding_box_centroid_));
        filter_->SetInitialVelocity(Eigen::Vector<double, 3>::Zero());
        filter_initialized_ = true;
    }

    // Do not advance the filter when there is no active detection.
    // Without a measurement to constrain the velocity, forward integration
    // would cause the position estimate to drift away from the last known
    // position indefinitely.
    if (state_ == TrackerState::NO_DETECTION) {
        if (publish_odometry_)     PublishOdometry();
        if (publish_pose_)         PublishPose();
        if (publish_detection_3d_) PublishDetection3D();
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
        (get_clock()->now() - last_valid_target_time_).seconds();
    if (has_tracked_target_ && dt_since_lidar > 2.0 / tracking_rate_) {
        if (publish_odometry_)     PublishOdometry();
        if (publish_pose_)         PublishPose();
        if (publish_detection_3d_) PublishDetection3D();
        return;
    }

    // Run the "Predict" step of the Kalman filter.
    filter_->Predict();

    // Check if a new measurement is available to be parsed, then provide a
    // matching measurement vector z_n.
    if (has_new_measurement_) {
        bounding_box_centroid_global_ = TransformToCoordinates(
            camera_frame_, world_frame_, bounding_box_centroid_);

        // The IMM measurement vector is 3D [x, y, z].  Velocity and
        // acceleration are estimated internally by each sub-filter.
        Eigen::Matrix<double, 3, 1> measurement_vector;
        measurement_vector(0) = bounding_box_centroid_global_.x();
        measurement_vector(1) = bounding_box_centroid_global_.y();
        measurement_vector(2) = bounding_box_centroid_global_.z();
		if (state_ == TrackerState::CAMERA_ONLY_TRACKING) {
			geometry_msgs::msg::TransformStamped transform_message;
			try {
				transform_message = transform_buffer_->lookupTransform(
					world_frame_.c_str(), camera_frame_.c_str(), tf2::TimePointZero);
                Eigen::Quaterniond q;
                tf2::fromMsg(transform_message.transform.rotation, q);
                Eigen::Matrix3d RotMatrix = q.toRotationMatrix();
                Eigen::Matrix3d R = RotMatrix * R_rdf_ * RotMatrix.transpose();

				// Run the IMM update step
				// with custom R.
				filter_->Update(measurement_vector,R);

			}
			catch (tf2::TransformException& exception) {
				RCLCPP_ERROR(get_logger(), "Transform lookup exception.");
			}

		}
		else {
			// Run the IMM update step.
			filter_->Update(measurement_vector);
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

    if (publish_odometry_) {
        PublishOdometry();
    }
    if (publish_pose_) {
        PublishPose();
    }
    if (publish_detection_3d_) {
        PublishDetection3D();
    }
}

void ObjectTrackingNode::PublishPointCloud(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, const rclcpp::Time& stamp,
    const std::string& frame_id,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher) {
    // Convert the PCL XYZ point cloud to a ROS sensor_msgs/PointCloud2
    // message.
    sensor_msgs::msg::PointCloud2 point_cloud_message;
    pcl::toROSMsg(*point_cloud, point_cloud_message);

    // Stamp the point cloud message with the provided header.
    point_cloud_message.header.stamp = stamp;
    point_cloud_message.header.frame_id = frame_id;

    publisher->publish(point_cloud_message);
}

void ObjectTrackingNode::Initialize() {
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    filter_initialized_ = false;

    // Pre-seed object_size_ so that CropRegionOfInterest() has a valid
    // non-zero box on the very first LIDAR_ONLY_TRACKING tick. Without this,
    // Eigen default-initializes object_size_ to zero, the crop box collapses
    // to a single point, the cloud is empty, and clustering fails immediately.
    // The value is updated after each successful bounding box estimation, so
    // this seed only affects the first attempt.
    object_size_ = roi_bounding_box_3d_size_;

    // Configure the voxel grid filter.
    voxel_grid_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);

    // Configure the passthrough filter.
    passthrough_filter_.setFilterFieldName("z");

    // Configure the SAC segmentation filter.
    sac_segmentation_.setOptimizeCoefficients(true);
    sac_segmentation_.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    sac_segmentation_.setEpsAngle((M_PI / 180.0) * sac_segmentation_angle_);
    sac_segmentation_.setAxis(Eigen::Vector3f{0.0, 1.0, 0.0});
    sac_segmentation_.setMethodType(pcl::SAC_RANSAC);
    sac_segmentation_.setMaxIterations(sac_segmentation_max_iterations_);
    sac_segmentation_.setDistanceThreshold(sac_segmentation_threshold_);
    sac_segmentation_.setNumberOfThreads(0);

    // Initialize the IMM filter (CV + CTR + NM).
    filter_ = std::make_shared<avt_341::perception::filtering::IMMFilter>(
        1.0 / estimator_rate_,
        filter_process_variance_,
        filter_measurement_variance_,
        imm_cv_init_prob_,
        imm_ctr_init_prob_,
        imm_nm_init_prob_,
        imm_persistence_prob_);
    filter_->SetInitialPosition(Eigen::Vector3d::Zero());

    has_target_selection_ = (use_autostart_) ? true : false;
    target_class_ = (use_autostart_) ? autostart_target_class_ : "";

    tracked_obstacle_id_ = -1;
    has_obstacle_markers_ = false;
    last_lidar_world_pos_ = Eigen::Vector3d::Zero();
    last_lidar_seen_time_ = get_clock()->now();

    last_valid_detection_time_ = get_clock()->now();
    last_valid_target_time_ = get_clock()->now();
}

void ObjectTrackingNode::PublishImage() {
    if (!has_image_)
        return;

    auto image_copy = latest_image_->image.clone();

    cv::Vec3b& color = image_copy.at<cv::Vec3b>(0, 0);
    color[2] = 13;

    // Publish the detection image
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = get_clock()->now();
    cv_image.header.frame_id = camera_frame_;
    cv_image.encoding = "bgr8";
    cv_image.image = image_copy;
    image_publisher_->publish(*cv_image.toImageMsg());
}

void ObjectTrackingNode::GetOrientedBoundingBox(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
    pcl::PointXYZ& bounding_box_min, pcl::PointXYZ& bounding_box_max,
    pcl::PointXYZ& bounding_box_centroid,
    Eigen::Matrix3f& bounding_box_rotation) {
    moi_estimation_.setInputCloud(point_cloud);
    moi_estimation_.compute();

    if (!moi_estimation_.getOBB(bounding_box_min, bounding_box_max,
                                bounding_box_centroid, bounding_box_rotation)) {
        throw PCAException(
            "Could not estimate the oriented bounding box for "
            "the provided point cloud cluster.");
    }
}

void ObjectTrackingNode::PublishDetection3D() {
    vision_msgs::msg::ObjectHypothesisWithPose object_hypothesis_message;

    object_hypothesis_message.hypothesis.class_id = target_class_;
    object_hypothesis_message.hypothesis.score = detection_score_;

    vision_msgs::msg::Detection3D detection_message;

    detection_message.header.stamp = get_clock()->now();
    detection_message.header.frame_id = camera_frame_;

    detection_message.results.push_back(object_hypothesis_message);

    detection_message.bbox.size.x = bounding_box_size_.x();
    detection_message.bbox.size.y = bounding_box_size_.y();
    detection_message.bbox.size.z = bounding_box_size_.z();

    detection_message.bbox.center.position.x = bounding_box_centroid_global_.x();
    detection_message.bbox.center.position.y = bounding_box_centroid_global_.y();
    detection_message.bbox.center.position.z = bounding_box_centroid_global_.z();

    detection_message.bbox.center.orientation.w = bounding_box_orientation_.w();
    detection_message.bbox.center.orientation.x = bounding_box_orientation_.x();
    detection_message.bbox.center.orientation.y = bounding_box_orientation_.y();
    detection_message.bbox.center.orientation.z = bounding_box_orientation_.z();

    detection_publisher_->publish(detection_message);
}

void ObjectTrackingNode::PublishPose() {
    if (use_filtered_pose_) {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_filtered_message;
        pose_filtered_message.header.stamp = get_clock()->now();
        pose_filtered_message.header.frame_id = world_frame_;
        pose_filtered_message.pose.pose.position.x = bounding_box_centroid_filtered_.x();
        pose_filtered_message.pose.pose.position.y = bounding_box_centroid_filtered_.y();
        pose_filtered_message.pose.pose.position.z = bounding_box_centroid_filtered_.z();
        pose_filtered_message.pose.pose.orientation.w = bounding_box_orientation_.w();
        pose_filtered_message.pose.pose.orientation.x = bounding_box_orientation_.x();
        pose_filtered_message.pose.pose.orientation.y = bounding_box_orientation_.y();
        pose_filtered_message.pose.pose.orientation.z = bounding_box_orientation_.z();
        pose_filtered_publisher_->publish(pose_filtered_message);
    }

    geometry_msgs::msg::PoseWithCovarianceStamped pose_message;
    pose_message.header.stamp = get_clock()->now();
    pose_message.header.frame_id = world_frame_;
    pose_message.pose.pose.position.x = bounding_box_centroid_global_.x();
    pose_message.pose.pose.position.y = bounding_box_centroid_global_.y();
    pose_message.pose.pose.position.z = bounding_box_centroid_global_.z();
    pose_message.pose.pose.orientation.w = bounding_box_orientation_.w();
    pose_message.pose.pose.orientation.x = bounding_box_orientation_.x();
    pose_message.pose.pose.orientation.y = bounding_box_orientation_.y();
    pose_message.pose.pose.orientation.z = bounding_box_orientation_.z();
    pose_publisher_->publish(pose_message);
}

void ObjectTrackingNode::PublishOdometry() {
    // Note that the filter does not predict orientation, hence the orientation
    // fields in the odometry message pose entry are not populated.

    if (use_filtered_odometry_) {
        nav_msgs::msg::Odometry odometry_filtered_message;
        odometry_filtered_message.header.stamp = get_clock()->now();
        odometry_filtered_message.header.frame_id = world_frame_;
        odometry_filtered_message.child_frame_id = odometry_child_frame_;
        odometry_filtered_message.pose.pose.position.x =
            bounding_box_centroid_filtered_.x();
        odometry_filtered_message.pose.pose.position.y =
            bounding_box_centroid_filtered_.y();
        odometry_filtered_message.pose.pose.position.z =
            bounding_box_centroid_filtered_.z();
        tf2::Quaternion q;
        q.setRPY(0, 0, filter_->GetYaw());
        // CTR state is now [x, vx, y, vy, omega]:
        //   position x at index 0, position y at index 2.
        Eigen::Matrix<double, 6, 6> OdometryCovariance =
            Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 5, 5> P = filter_->GetCTRCovariance();
        OdometryCovariance(0, 0) = P(0, 0);  // x variance
        OdometryCovariance(0, 1) = P(0, 2);  // xy covariance
        OdometryCovariance(1, 0) = P(2, 0);
        OdometryCovariance(1, 1) = P(2, 2);  // y variance
        OdometryCovariance(2, 2) = 100.0;    // no information on z
        OdometryCovariance(3, 3) = 9.0;      // no information on roll
        OdometryCovariance(4, 4) = 9.0;      // no information on pitch
        OdometryCovariance(5, 5) = filter_->GetFusedYawVariance();
        //odometry_filtered_message.pose.pose.orientation = tf2::toMsg(q);
        odometry_filtered_message.pose.pose.orientation.x = 0;
        odometry_filtered_message.pose.pose.orientation.y = 0;
        odometry_filtered_message.pose.pose.orientation.z = sin(filter_->GetYaw() / 2);
        odometry_filtered_message.pose.pose.orientation.w = cos(filter_->GetYaw() / 2);
        // odometry_filtered_message.pose.pose.orientation.normalise();
        for (size_t i = 0; i < 6; i++)  {
            for (size_t j = 0; j < 6; j++) {
                odometry_filtered_message.pose.covariance[i * 6 + j] = OdometryCovariance(i, j);
            }
        }
        odometry_filtered_publisher_->publish(odometry_filtered_message);
    }

    nav_msgs::msg::Odometry odometry_message;
    odometry_message.header.stamp = get_clock()->now();
    odometry_message.header.frame_id = world_frame_;
    odometry_message.child_frame_id = odometry_child_frame_;
    odometry_message.pose.pose.position.x = bounding_box_centroid_global_.x();
    odometry_message.pose.pose.position.y = bounding_box_centroid_global_.y();
    odometry_message.pose.pose.position.z = bounding_box_centroid_global_.z();
   
    odometry_publisher_->publish(odometry_message);
}

void ObjectTrackingNode::TaskStatusCallback(
    avt_341_msgs::msg::MissionTaskStatus::SharedPtr task_status_message) {
    target_class_ = task_status_message->tracked_vehicle;
    has_target_selection_ = true;

    std::string message("Target selection set to \"" + target_class_ + "\".");
    RCLCPP_INFO(get_logger(), message.c_str());
}

rcl_interfaces::msg::SetParametersResult
ObjectTrackingNode::SetParametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
    for (const auto& parameter : parameters) {
        if (parameter.get_name() == "camera_frame") {
            camera_frame_ = parameter.as_string();
        } else if (parameter.get_name() == "world_frame") {
            world_frame_ = parameter.as_string();
        } else if (parameter.get_name() == "odometry_child_frame") {
            odometry_child_frame_ = parameter.as_string();
        } else if (parameter.get_name() == "filters_downsampling_leaf_size") {
            leaf_size_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_passthrough_min") {
            passthrough_distance_min_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_passthrough_max") {
            passthrough_distance_max_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_clustering_tolerance") {
            clustering_tolerance_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_clustering_size_minimum") {
            cluster_size_min_ = parameter.as_int();
        } else if (parameter.get_name() == "filters_clustering_size_maximum") {
            cluster_size_max_ = parameter.as_int();
        } else if (parameter.get_name() == "filters_clustering_min_height") {
            cluster_height_min_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_clustering_max_height") {
            cluster_height_max_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_clustering_min_width") {
            cluster_width_min_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_clustering_max_width") {
            cluster_width_max_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_clustering_min_depth") {
            cluster_depth_min_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_clustering_max_depth") {
            cluster_depth_max_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_clustering_distance_reference") {
            cluster_distance_ref_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_ground_max_iterations") {
            sac_segmentation_max_iterations_ = parameter.as_int();
        } else if (parameter.get_name() == "filters_ground_threshold") {
            sac_segmentation_threshold_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_ground_angle") {
            sac_segmentation_angle_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_roi_scale_factor") {
            roi_scale_factor_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_kalman_rate") {
            estimator_rate_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_kalman_process") {
            filter_process_variance_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_kalman_measurement") {
            filter_measurement_variance_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_use_pca_centroid") {
            use_pca_centroid_ = parameter.as_bool();
        } else if (parameter.get_name() == "tracker_timeout") {
            target_timeout_ = parameter.as_double();
        } else if (parameter.get_name() == "sync_enable") {
            sync_messages_ = parameter.as_bool();
        } else if (parameter.get_name() == "sync_use_callback") {
            use_callback_time_ = parameter.as_bool();
        } else if (parameter.get_name() == "sync_detection") {
            max_detection_skew_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_pose") {
            use_filtered_pose_ = parameter.as_bool();
        } else if (parameter.get_name() == "filters_odometry") {
            use_filtered_odometry_ = parameter.as_bool();
        } else if (parameter.get_name() == "filters_use_manual_roi") {
            use_manual_roi_size_ = parameter.as_bool();
        }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

std::string ObjectTrackingNode::ToString(TrackerState& state) {
    if (state == TrackerState::UNINITIALIZED) {
        return "uninitialized";
    } else if (state == TrackerState::INACTIVE) {
        return "inactive";
    } else if (state == TrackerState::NO_DETECTION) {
        return "no_detection";
    } else if (state == TrackerState::LIDAR_ONLY_TRACKING) {
        return "lidar_only";
    } else if (state == TrackerState::FULL_TRACKING) {
        return "full";
    }
    throw std::invalid_argument("Unknown tracker state.");
}

void ObjectTrackingNode::SetTargetServiceCallback(
    const std::shared_ptr<avt_341_msgs::srv::SetTarget::Request> request,
    std::shared_ptr<avt_341_msgs::srv::SetTarget::Response> response) {
    // Suppress unused variable compiler warning.
    (void)request;

    target_class_ = request->id;
    has_target_selection_ = true;

    std::string message("Target selection set to \"" + target_class_ + "\".");
    RCLCPP_INFO(get_logger(), message.c_str());
    response->success = true;
    response->message = message.c_str();
}

void ObjectTrackingNode::ProjectPointsToPixel(
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message) {
    // Find the mapping between the 3D coordinates (X, Y, Z) of a point and
    // the corresponding pixel coordinates in the camera image frame.
    const std::vector<PixelCoordinates> coordinates =
        ConvertPointCloudToPixelCoordinates(point_cloud, camera_info_message_);

    // Find the cloud points in the camera field of view and store them in a
    // separate point cloud. We keep the original downsampled point cloud to
    // account for objects that are not yet in the field-of-view.
    FindPointsInCameraFOV(point_cloud, coordinates,
                          camera_info_message_->height,
                          camera_info_message_->width);

    if (publish_fov_cloud_) {
        PublishPointCloud(point_cloud, point_cloud_message->header.stamp,
                          camera_frame_, fov_cloud_publisher_);
    }

    // Find cloud points the region of interest defined by the first
    // detection.

    RCLCPP_DEBUG(get_logger(),
                 "Finding cloud points in the region of interest ...");

    const double cx = detections_message_.bbox.center.position.x;
    const double cy = detections_message_.bbox.center.position.y;
    const double half_roi_w = detections_message_.bbox.size_x / 2;
    const double half_roi_h = detections_message_.bbox.size_y / 2;
    const double w = camera_info_message_->width;
    const double h = camera_info_message_->height;

    auto x_min = static_cast<unsigned int>(std::clamp(cx - half_roi_w, 0.0, w));
    auto x_max = static_cast<unsigned int>(std::clamp(std::ceil(cx + half_roi_w), 0.0, w));
    auto y_min = static_cast<unsigned int>(std::clamp(cy - half_roi_h, 0.0, h));
    auto y_max = static_cast<unsigned int>(std::clamp(std::ceil(cy + half_roi_h), 0.0, h));

    RCLCPP_DEBUG(get_logger(),
                 "Selected the following bounding box as region of interest: "
                 "[X_MIN: %u, X_MAX: %u Y_MIN: %u Y_MAX: %u]",
                 x_min, x_max, y_min, y_max);

    // Find the points lying in the region of interest (ROI) defined by the
    // detection bounding box.
    FindPointsInROI(point_cloud, coordinates, x_min, x_max, y_min, y_max);

    if (publish_roi_cloud_) {
        PublishPointCloud(point_cloud, point_cloud_message->header.stamp,
                          camera_frame_, roi_cloud_publisher_);
    }

    RemoveNaNPoints(point_cloud);
}

}  // namespace perception
}  // namespace avt_341
