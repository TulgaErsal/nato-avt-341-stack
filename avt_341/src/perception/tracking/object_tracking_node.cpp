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

    declare_parameter("publish_pose", true);
    publish_pose_ = get_parameter("publish_pose").as_bool();

    declare_parameter("filters_pose", true);
    use_filtered_pose_ = get_parameter("filters_pose").as_bool();

    declare_parameter("publish_odometry", false);
    publish_odometry_ = get_parameter("publish_odometry").as_bool();

    declare_parameter("filters_odometry", true);
    use_filtered_odometry_ = get_parameter("filters_odometry").as_bool();

    declare_parameter("heading_min_speed", 0.5);
    heading_min_speed_ = get_parameter("heading_min_speed").as_double();

    declare_parameter("heading_resume_speed", 1.0);
    heading_resume_speed_ = get_parameter("heading_resume_speed").as_double();

    declare_parameter("publish_detection", false);
    publish_detection_3d_ = get_parameter("publish_detection").as_bool();

    declare_parameter("publish_image", false);
    publish_image_ = get_parameter("publish_image").as_bool();

    declare_parameter("filters_use_manual_roi", false);
    use_manual_roi_size_ = get_parameter("filters_use_manual_roi").as_bool();

    declare_parameter("obstacle_association_max_dist", 5.0);
    obstacle_association_max_dist_ =
        get_parameter("obstacle_association_max_dist").as_double();

    // Integrated LiDAR obstacle detector parameters
    declare_parameter("od_robot_base_link", std::string("base_link"));
    od_robot_base_link_ = get_parameter("od_robot_base_link").as_string();
    od_robot_base_link_ = frame_prefix + od_robot_base_link_;

    declare_parameter("od_use_pca_box", false);
    od_use_pca_box_ = get_parameter("od_use_pca_box").as_bool();

    declare_parameter("od_use_tracking", true);
    od_use_tracking_ = get_parameter("od_use_tracking").as_bool();

    declare_parameter("od_voxel_grid_size", 0.2);
    od_voxel_grid_size_ =
        static_cast<float>(get_parameter("od_voxel_grid_size").as_double());

    declare_parameter("od_roi_max_x", 70.0);
    declare_parameter("od_roi_max_y", 30.0);
    declare_parameter("od_roi_max_z", 3.0);
    declare_parameter("od_roi_min_x", -5.0);
    declare_parameter("od_roi_min_y", -30.0);
    declare_parameter("od_roi_min_z", -2.5);
    od_roi_max_point_ = Eigen::Vector4f(
        get_parameter("od_roi_max_x").as_double(),
        get_parameter("od_roi_max_y").as_double(),
        get_parameter("od_roi_max_z").as_double(), 1.0f);
    od_roi_min_point_ = Eigen::Vector4f(
        get_parameter("od_roi_min_x").as_double(),
        get_parameter("od_roi_min_y").as_double(),
        get_parameter("od_roi_min_z").as_double(), 1.0f);

    declare_parameter("od_body_max_x", 0.3);
    declare_parameter("od_body_max_y", 0.8);
    declare_parameter("od_body_max_z", 2.0);
    declare_parameter("od_body_min_x", -2.2);
    declare_parameter("od_body_min_y", -0.8);
    declare_parameter("od_body_min_z", -0.3);
    od_body_max_point_ = Eigen::Vector4f(
        get_parameter("od_body_max_x").as_double(),
        get_parameter("od_body_max_y").as_double(),
        get_parameter("od_body_max_z").as_double(), 1.0f);
    od_body_min_point_ = Eigen::Vector4f(
        get_parameter("od_body_min_x").as_double(),
        get_parameter("od_body_min_y").as_double(),
        get_parameter("od_body_min_z").as_double(), 1.0f);

    declare_parameter("od_ground_normal_x", 0.0);
    declare_parameter("od_ground_normal_y", 0.0);
    declare_parameter("od_ground_normal_z", 1.0);
    od_ground_normal_ = Eigen::Vector3f(
        get_parameter("od_ground_normal_x").as_double(),
        get_parameter("od_ground_normal_y").as_double(),
        get_parameter("od_ground_normal_z").as_double());

    declare_parameter("od_ground_normal_threshold", 0.4);
    od_ground_normal_threshold_ = static_cast<float>(
        get_parameter("od_ground_normal_threshold").as_double());

    declare_parameter("od_obstacle_scale", 1.0);
    od_obstacle_scale_ =
        static_cast<float>(get_parameter("od_obstacle_scale").as_double());

    declare_parameter("od_obstacle_min_neighbors", 10);
    od_obstacle_min_neighbors_ =
        get_parameter("od_obstacle_min_neighbors").as_int();

    declare_parameter("od_cluster_threshold", 0.6);
    od_cluster_threshold_ = static_cast<float>(
        get_parameter("od_cluster_threshold").as_double());

    declare_parameter("od_cluster_min_size", 10);
    od_cluster_min_size_ = get_parameter("od_cluster_min_size").as_int();

    declare_parameter("od_cluster_max_size", 5000);
    od_cluster_max_size_ = get_parameter("od_cluster_max_size").as_int();

    declare_parameter("od_displacement_threshold", 1.0);
    od_displacement_threshold_ = static_cast<float>(
        get_parameter("od_displacement_threshold").as_double());

    declare_parameter("od_iou_threshold", 1.0);
    od_iou_threshold_ =
        static_cast<float>(get_parameter("od_iou_threshold").as_double());

    declare_parameter("od_publish_ground_cloud", false);
    od_publish_ground_cloud_ =
        get_parameter("od_publish_ground_cloud").as_bool();

    declare_parameter("od_publish_cluster_cloud", false);
    od_publish_cluster_cloud_ =
        get_parameter("od_publish_cluster_cloud").as_bool();

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

    // Initialize the integrated obstacle detector.
    obstacle_detector_ =
        std::make_shared<avt_341::perception::LidarObstacleDetector<pcl::PointXYZ>>();
    obstacle_id_ = 0;
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

    tracked_target_odometry_publisher_ =
        create_publisher<nav_msgs::msg::Odometry>("avt_341/odometry/tracked", 1);

    info_publisher_ =
        create_publisher<avt_341_msgs::msg::TrackerInfo>("info", 1);

    // Integrated obstacle detector publishers.
    obstacle_bboxes_publisher_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "lidar_detector/bboxes", 1);

    if (od_publish_ground_cloud_) {
        obstacle_ground_cloud_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                "lidar_detector/cloud_ground", 1);
    }

    if (od_publish_cluster_cloud_) {
        obstacle_clusters_cloud_publisher_ =
            create_publisher<sensor_msgs::msg::PointCloud2>(
                "lidar_detector/cloud_clusters", 1);
    }
}

void ObjectTrackingNode::TrackerInfoCallback() {
    avt_341_msgs::msg::TrackerInfo info_message;
    info_message.header.stamp = get_clock()->now();
    info_message.header.frame_id = "none";
    info_message.state = state_;
    info_publisher_->publish(info_message);
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

void ObjectTrackingNode::Reset() {
    execution_time_ = -1.0;
}

void ObjectTrackingNode::PublishObstacleDeleteAll(
    const std_msgs::msg::Header& header) {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_marker.header = header;
    marker_array.markers.push_back(delete_marker);
    latest_obstacle_markers_ = marker_array;
    has_obstacle_markers_ = true;
    obstacle_bboxes_publisher_->publish(marker_array);
}

void ObjectTrackingNode::PublishObstacleMarkers(
    const std_msgs::msg::Header& header) {
    visualization_msgs::msg::MarkerArray marker_array;

    // Always prepend a DELETEALL to clear stale markers from previous frames.
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_marker.header = header;
    marker_array.markers.push_back(delete_marker);

    for (const auto& box : curr_boxes_) {
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "lidar_bboxes";
        marker.id = box.id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = box.position.x();
        marker.pose.position.y = box.position.y();
        marker.pose.position.z = box.position.z();
        marker.pose.orientation.w = box.quaternion.w();
        marker.pose.orientation.x = box.quaternion.x();
        marker.pose.orientation.y = box.quaternion.y();
        marker.pose.orientation.z = box.quaternion.z();
        marker.scale.x = box.dimension.x();
        marker.scale.y = box.dimension.y();
        marker.scale.z = box.dimension.z();
        marker.color.r = 1.0f;
        marker.color.g = 0.5f;
        marker.color.b = 0.0f;
        marker.color.a = 0.3f;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker_array.markers.push_back(marker);
    }

    latest_obstacle_markers_ = marker_array;
    has_obstacle_markers_ = true;
    obstacle_bboxes_publisher_->publish(marker_array);
}

void ObjectTrackingNode::RunObstacleDetection(
    const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg) {
    std_msgs::msg::Header header = cloud_msg->header;

    // Transform to robot base link if needed, using the cloud's own timestamp
    // so that the TF lookup matches the moment the scan was captured (same as
    // the original standalone obstacle detector node).
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    if (cloud_msg->header.frame_id != od_robot_base_link_) {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = transform_buffer_->lookupTransform(
                od_robot_base_link_, cloud_msg->header.frame_id,
                cloud_msg->header.stamp, tf2::durationFromSec(0.2));
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(get_logger(),
                        "RunObstacleDetection: TF %s -> %s failed: %s",
                        cloud_msg->header.frame_id.c_str(),
                        od_robot_base_link_.c_str(), ex.what());
            PublishObstacleDeleteAll(header);
            return;
        }
        tf2::doTransform(*cloud_msg, transformed_cloud, tf);
        transformed_cloud.header.frame_id = od_robot_base_link_;
    } else {
        transformed_cloud = *cloud_msg;
    }
    header.frame_id = od_robot_base_link_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(transformed_cloud, *raw_cloud);

    // Downsample, crop ROI, remove ego-vehicle body.
    auto filtered_cloud = obstacle_detector_->filterCloud(
        raw_cloud, od_voxel_grid_size_,
        od_roi_min_point_, od_roi_max_point_,
        od_body_min_point_, od_body_max_point_);

    if (static_cast<int>(filtered_cloud->size()) < od_cluster_min_size_) {
        PublishObstacleDeleteAll(header);
        return;
    }

    // Rotation-only transform to fixed frame for normal estimation.
    // Use the cloud's timestamp to match the original standalone node behavior.
    pcl::PointCloud<pcl::PointXYZ>::Ptr fixed_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    geometry_msgs::msg::TransformStamped fixed_tf;
    try {
        fixed_tf = transform_buffer_->lookupTransform(
            world_frame_, od_robot_base_link_,
            cloud_msg->header.stamp, tf2::durationFromSec(0.2));
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(get_logger(),
                    "RunObstacleDetection: fixed-frame TF not available: %s",
                    ex.what());
        PublishObstacleDeleteAll(header);
        return;
    }

    Eigen::Quaternionf q(
        fixed_tf.transform.rotation.w, fixed_tf.transform.rotation.x,
        fixed_tf.transform.rotation.y, fixed_tf.transform.rotation.z);
    if (q.norm() < 1e-6f) {
        RCLCPP_WARN(get_logger(),
                    "RunObstacleDetection: fixed-frame TF quaternion is zero.");
        PublishObstacleDeleteAll(header);
        return;
    }
    Eigen::Matrix4f mat4 = Eigen::Matrix4f::Identity();
    mat4.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    Eigen::Affine3f transform_fixed;
    transform_fixed.matrix() = mat4;
    pcl::transformPointCloud(*filtered_cloud, *fixed_cloud, transform_fixed);

    if (static_cast<int>(fixed_cloud->size()) < od_cluster_min_size_) {
        PublishObstacleDeleteAll(header);
        return;
    }

    // Separate ground and obstacle points via normal filtering.
    pcl::PointCloud<pcl::PointXYZ>::Ptr norm_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);
    obstacle_detector_->pclFilterNorms(
        filtered_cloud, fixed_cloud,
        norm_filtered, ground_filtered,
        od_ground_normal_, od_ground_normal_threshold_,
        od_obstacle_scale_, od_obstacle_min_neighbors_);

    if (od_publish_ground_cloud_ && obstacle_ground_cloud_publisher_) {
        sensor_msgs::msg::PointCloud2 ground_msg;
        pcl::toROSMsg(*ground_filtered, ground_msg);
        ground_msg.header = header;
        obstacle_ground_cloud_publisher_->publish(ground_msg);
    }

    if (od_publish_cluster_cloud_ && obstacle_clusters_cloud_publisher_) {
        sensor_msgs::msg::PointCloud2 cluster_msg;
        pcl::toROSMsg(*norm_filtered, cluster_msg);
        cluster_msg.header = header;
        obstacle_clusters_cloud_publisher_->publish(cluster_msg);
    }

    if (norm_filtered->empty()) {
        PublishObstacleDeleteAll(header);
        return;
    }

    // Cluster and build bounding boxes.
    auto cloud_clusters = obstacle_detector_->clustering(
        norm_filtered, od_cluster_threshold_,
        od_cluster_min_size_, od_cluster_max_size_);

    curr_boxes_.clear();
    for (auto& cluster : cloud_clusters) {
        Box box = od_use_pca_box_
            ? obstacle_detector_->pcaBoundingBox(cluster, obstacle_id_)
            : obstacle_detector_->axisAlignedBoundingBox(cluster, obstacle_id_);
        obstacle_id_ = (obstacle_id_ < SIZE_MAX) ? obstacle_id_ + 1 : 0;
        curr_boxes_.emplace_back(box);
    }

    if (od_use_tracking_) {
        obstacle_detector_->obstacleTracking(
            prev_boxes_, curr_boxes_,
            od_displacement_threshold_, od_iou_threshold_);
    }

    PublishObstacleMarkers(header);

    prev_boxes_.swap(curr_boxes_);
    curr_boxes_.clear();
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

    point_cloud_message_ = point_cloud_message;
    has_point_cloud_ = true;

    // Run the integrated obstacle detector synchronously so that
    // latest_obstacle_markers_ is up-to-date before the next tracking tick.
    RunObstacleDetection(point_cloud_message);
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
				// with custom R. if chi2 is acceptable
                double chi2 = filter_->GetChi2IMM2D(measurement_vector, R);
                // Hard 4-sigma treshold TODO open up for soft and as parameter
                if (chi2 <4*4) 
				    filter_->Update(measurement_vector,R);
                else {
                    const auto state_filtered = filter_->GetState();

                    RCLCPP_WARN(get_logger(),
                        "chi2 test failed chi2 at %.2lf z(0)-x = %.2lf, z(1)-y = %.2lf current detection, skipping ...",
                        chi2, measurement_vector(0) - state_filtered(0), measurement_vector(1) - state_filtered(3) );
                    state_ = TrackerState::NO_DETECTION;
                }


			}
			catch (tf2::TransformException& exception) {
				RCLCPP_ERROR(get_logger(), "Transform lookup exception.");
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

                RCLCPP_WARN(get_logger(),
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

void ObjectTrackingNode::UpdateHeadingHold() {
    const double speed = filter_->GetCTRSpeed();
    if (!heading_held_ && speed < heading_min_speed_) {
        heading_held_ = true;
    } else if (heading_held_ && speed >= heading_resume_speed_) {
        heading_held_ = false;
    }
    if (!heading_held_) {
        last_reliable_yaw_ = filter_->GetYaw();
    }
}

void ObjectTrackingNode::PublishPose() {
    UpdateHeadingHold();

    if (use_filtered_pose_) {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_filtered_message;
        pose_filtered_message.header.stamp = get_clock()->now();
        pose_filtered_message.header.frame_id = world_frame_;
        pose_filtered_message.pose.pose.position.x = bounding_box_centroid_filtered_.x();
        pose_filtered_message.pose.pose.position.y = bounding_box_centroid_filtered_.y();
        pose_filtered_message.pose.pose.position.z = bounding_box_centroid_filtered_.z();
        pose_filtered_message.pose.pose.orientation.x = 0;
        pose_filtered_message.pose.pose.orientation.y = 0;
        pose_filtered_message.pose.pose.orientation.z = sin(last_reliable_yaw_ / 2);
        pose_filtered_message.pose.pose.orientation.w = cos(last_reliable_yaw_ / 2);
        Eigen::Matrix<double, 6, 6> PoseCovariance =
            Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 5, 5> P = filter_->GetCTRCovariance();
        PoseCovariance(0, 0) = P(0, 0);  // x variance
        PoseCovariance(0, 1) = P(0, 2);  // xy covariance
        PoseCovariance(1, 0) = P(2, 0);
        PoseCovariance(1, 1) = P(2, 2);  // y variance
        PoseCovariance(2, 2) = 100.0;    // no information on z
        PoseCovariance(3, 3) = 9.0;      // no information on roll
        PoseCovariance(4, 4) = 9.0;      // no information on pitch
        PoseCovariance(5, 5) = filter_->GetFusedYawVariance();
        for (size_t i = 0; i < 6; i++) {
            for (size_t j = 0; j < 6; j++) {
                pose_filtered_message.pose.covariance[i * 6 + j] = PoseCovariance(i, j);
            }
        }
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
    UpdateHeadingHold();

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
        odometry_filtered_message.pose.pose.orientation.x = 0;
        odometry_filtered_message.pose.pose.orientation.y = 0;
        odometry_filtered_message.pose.pose.orientation.z = sin(last_reliable_yaw_ / 2);
        odometry_filtered_message.pose.pose.orientation.w = cos(last_reliable_yaw_ / 2);
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

    nav_msgs::msg::Odometry tracked_target_message;
    tracked_target_message.header.stamp = get_clock()->now();
    tracked_target_message.header.frame_id = world_frame_;
    tracked_target_message.child_frame_id = odometry_child_frame_;
    tracked_target_message.pose.pose.position.x = bounding_box_centroid_filtered_.x();
    tracked_target_message.pose.pose.position.y = bounding_box_centroid_filtered_.y();
    tracked_target_message.pose.pose.position.z = bounding_box_centroid_filtered_.z();
    tracked_target_message.pose.pose.orientation.x = 0;
    tracked_target_message.pose.pose.orientation.y = 0;
    tracked_target_message.pose.pose.orientation.z = sin(last_reliable_yaw_ / 2);
    tracked_target_message.pose.pose.orientation.w = cos(last_reliable_yaw_ / 2);
    tracked_target_odometry_publisher_->publish(tracked_target_message);
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
        } else if (parameter.get_name() == "filters_roi_scale_factor") {
            roi_scale_factor_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_kalman_rate") {
            estimator_rate_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_kalman_process") {
            filter_process_variance_ = parameter.as_double();
        } else if (parameter.get_name() == "filters_kalman_measurement") {
            filter_measurement_variance_ = parameter.as_double();
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
        } else if (parameter.get_name() == "heading_min_speed") {
            heading_min_speed_ = parameter.as_double();
        } else if (parameter.get_name() == "heading_resume_speed") {
            heading_resume_speed_ = parameter.as_double();
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
    } else if (state == TrackerState::CAMERA_ONLY_TRACKING) {
        return "camera_only";
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

}  // namespace perception
}  // namespace avt_341
