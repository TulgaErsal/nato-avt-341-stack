/**
* @file      object_tracker_params.cpp
* @brief     ROS parameter declaration and parsing for the camera/LiDAR
             sensor fusion object tracker settings.
*/

#include <avt_341/perception/tracking/tracker_params.hpp>

namespace avt_341 {
namespace perception {

ObjectTrackerSettings ObjectTrackerSettings::Load(rclcpp::Node& node) {
    ObjectTrackerSettings s;

    node.declare_parameter("tracking_rate", 10.0);
    s.tracking.tracking_rate = node.get_parameter("tracking_rate").as_double();

    node.declare_parameter("info_rate", 1.0);
    s.tracking.info_rate = node.get_parameter("info_rate").as_double();

    node.declare_parameter("frame_prefix", "");
    const std::string frame_prefix =
        node.get_parameter("frame_prefix").as_string();

    node.declare_parameter("camera_frame", "camera_optical");
    s.frames.camera_frame =
        frame_prefix + node.get_parameter("camera_frame").as_string();

    node.declare_parameter("world_frame", "Q");
    s.frames.world_frame = node.get_parameter("world_frame").as_string();

    node.declare_parameter("odometry_child_frame", "odom");
    s.frames.odometry_child_frame =
        node.get_parameter("odometry_child_frame").as_string();

    node.declare_parameter("filters_roi_scale_factor", 1.5);
    s.filter.roi_scale_factor =
        node.get_parameter("filters_roi_scale_factor").as_double();

    node.declare_parameter("filters_kalman_rate", 50.0);
    s.filter.estimator_rate =
        node.get_parameter("filters_kalman_rate").as_double();

    node.declare_parameter("filters_kalman_process", 0.01);
    s.filter.process_variance =
        node.get_parameter("filters_kalman_process").as_double();

    node.declare_parameter("filters_kalman_measurement", 1.0);
    s.filter.measurement_variance =
        node.get_parameter("filters_kalman_measurement").as_double();

    node.declare_parameter("camera_target_height", 5.0);
    s.camera.target_height =
        node.get_parameter("camera_target_height").as_double();

    node.declare_parameter("camera_bbox_pixel_sigma", 4.0);
    s.camera.bbox_pixel_sigma =
        node.get_parameter("camera_bbox_pixel_sigma").as_double();

    node.declare_parameter("filters_imm_cv_init_prob", 0.33);
    s.filter.imm_cv_init_prob =
        node.get_parameter("filters_imm_cv_init_prob").as_double();

    node.declare_parameter("filters_imm_ctr_init_prob", 0.33);
    s.filter.imm_ctr_init_prob =
        node.get_parameter("filters_imm_ctr_init_prob").as_double();

    node.declare_parameter("filters_imm_nm_init_prob", 0.33);
    s.filter.imm_nm_init_prob =
        node.get_parameter("filters_imm_nm_init_prob").as_double();

    node.declare_parameter("filters_imm_persistence_prob", 0.9);
    s.filter.imm_persistence_prob =
        node.get_parameter("filters_imm_persistence_prob").as_double();

    node.declare_parameter("tracker_autostart", true);
    s.target_selection.use_autostart =
        node.get_parameter("tracker_autostart").as_bool();

    node.declare_parameter("tracker_use_mission_manager", true);
    s.target_selection.use_mission_manager =
        node.get_parameter("tracker_use_mission_manager").as_bool();

    node.declare_parameter("tracker_target_classes",
                           std::vector<std::string>{"mrzr4"});
    s.target_selection.autostart_target_classes =
        node.get_parameter("tracker_target_classes").as_string_array();

    node.declare_parameter("tracker_timeout", 5.0);
    s.tracking.target_timeout = node.get_parameter("tracker_timeout").as_double();

    node.declare_parameter("formation_vehicle_ids", std::vector<std::string>{});
    s.target_selection.formation_vehicle_ids =
        node.get_parameter("formation_vehicle_ids").as_string_array();

    node.declare_parameter("tracker_use_multi_tracking", false);
    s.target_selection.use_multi_tracking =
        node.get_parameter("tracker_use_multi_tracking").as_bool();

    node.declare_parameter("tracker_toi_regex", std::string("^TI_"));
    s.target_selection.toi_regex =
        node.get_parameter("tracker_toi_regex").as_string();

    node.declare_parameter("tracker_allow_generic", true);
    s.target_selection.allow_generic =
        node.get_parameter("tracker_allow_generic").as_bool();

    node.declare_parameter("sync_enable", true);
    s.sync.enabled = node.get_parameter("sync_enable").as_bool();

    node.declare_parameter("sync_use_callback", false);
    s.sync.use_callback_time = node.get_parameter("sync_use_callback").as_bool();

    node.declare_parameter("sync_detection", 0.1);
    s.sync.max_detection_skew = node.get_parameter("sync_detection").as_double();

    node.declare_parameter("publish_odometry", false);
    s.publish.odometry = node.get_parameter("publish_odometry").as_bool();

    node.declare_parameter("heading_min_speed", 0.5);
    s.tracking.heading_min_speed =
        node.get_parameter("heading_min_speed").as_double();

    node.declare_parameter("heading_resume_speed", 1.0);
    s.tracking.heading_resume_speed =
        node.get_parameter("heading_resume_speed").as_double();

    node.declare_parameter("publish_detection", false);
    s.publish.detection_3d = node.get_parameter("publish_detection").as_bool();

    node.declare_parameter("publish_image", false);
    s.publish.image = node.get_parameter("publish_image").as_bool();

    node.declare_parameter("filters_use_manual_roi", false);
    s.filter.use_manual_roi =
        node.get_parameter("filters_use_manual_roi").as_bool();

    node.declare_parameter("obstacle_association_max_dist", 5.0);
    s.tracking.obstacle_association_max_dist =
        node.get_parameter("obstacle_association_max_dist").as_double();

    // Integrated LiDAR obstacle detector parameters
    node.declare_parameter("od_robot_base_link", std::string("base_link"));
    s.obstacle_detector.robot_base_link =
        frame_prefix + node.get_parameter("od_robot_base_link").as_string();

    node.declare_parameter("od_use_pca_box", false);
    s.obstacle_detector.use_pca_box =
        node.get_parameter("od_use_pca_box").as_bool();

    node.declare_parameter("od_use_tracking", true);
    s.obstacle_detector.use_tracking =
        node.get_parameter("od_use_tracking").as_bool();

    node.declare_parameter("od_voxel_grid_size", 0.2);
    s.obstacle_detector.voxel_grid_size =
        static_cast<float>(node.get_parameter("od_voxel_grid_size").as_double());

    node.declare_parameter("od_roi_max_x", 70.0);
    node.declare_parameter("od_roi_max_y", 30.0);
    node.declare_parameter("od_roi_max_z", 3.0);
    node.declare_parameter("od_roi_min_x", -5.0);
    node.declare_parameter("od_roi_min_y", -30.0);
    node.declare_parameter("od_roi_min_z", -2.5);
    s.obstacle_detector.roi_max_point = Eigen::Vector4f(
        node.get_parameter("od_roi_max_x").as_double(),
        node.get_parameter("od_roi_max_y").as_double(),
        node.get_parameter("od_roi_max_z").as_double(), 1.0f);
    s.obstacle_detector.roi_min_point = Eigen::Vector4f(
        node.get_parameter("od_roi_min_x").as_double(),
        node.get_parameter("od_roi_min_y").as_double(),
        node.get_parameter("od_roi_min_z").as_double(), 1.0f);

    node.declare_parameter("od_body_max_x", 0.3);
    node.declare_parameter("od_body_max_y", 0.8);
    node.declare_parameter("od_body_max_z", 2.0);
    node.declare_parameter("od_body_min_x", -2.2);
    node.declare_parameter("od_body_min_y", -0.8);
    node.declare_parameter("od_body_min_z", -0.3);
    s.obstacle_detector.body_max_point = Eigen::Vector4f(
        node.get_parameter("od_body_max_x").as_double(),
        node.get_parameter("od_body_max_y").as_double(),
        node.get_parameter("od_body_max_z").as_double(), 1.0f);
    s.obstacle_detector.body_min_point = Eigen::Vector4f(
        node.get_parameter("od_body_min_x").as_double(),
        node.get_parameter("od_body_min_y").as_double(),
        node.get_parameter("od_body_min_z").as_double(), 1.0f);

    node.declare_parameter("od_ground_normal_x", 0.0);
    node.declare_parameter("od_ground_normal_y", 0.0);
    node.declare_parameter("od_ground_normal_z", 1.0);
    s.obstacle_detector.ground_normal = Eigen::Vector3f(
        node.get_parameter("od_ground_normal_x").as_double(),
        node.get_parameter("od_ground_normal_y").as_double(),
        node.get_parameter("od_ground_normal_z").as_double());

    node.declare_parameter("od_ground_normal_threshold", 0.4);
    s.obstacle_detector.ground_normal_threshold = static_cast<float>(
        node.get_parameter("od_ground_normal_threshold").as_double());

    node.declare_parameter("od_obstacle_scale", 1.0);
    s.obstacle_detector.obstacle_scale =
        static_cast<float>(node.get_parameter("od_obstacle_scale").as_double());

    node.declare_parameter("od_obstacle_min_neighbors", 10);
    s.obstacle_detector.obstacle_min_neighbors =
        node.get_parameter("od_obstacle_min_neighbors").as_int();

    node.declare_parameter("od_cluster_threshold", 0.6);
    s.obstacle_detector.cluster_threshold = static_cast<float>(
        node.get_parameter("od_cluster_threshold").as_double());

    node.declare_parameter("od_cluster_min_size", 10);
    s.obstacle_detector.cluster_min_size =
        node.get_parameter("od_cluster_min_size").as_int();

    node.declare_parameter("od_cluster_max_size", 5000);
    s.obstacle_detector.cluster_max_size =
        node.get_parameter("od_cluster_max_size").as_int();

    node.declare_parameter("od_displacement_threshold", 1.0);
    s.obstacle_detector.displacement_threshold = static_cast<float>(
        node.get_parameter("od_displacement_threshold").as_double());

    node.declare_parameter("od_iou_threshold", 1.0);
    s.obstacle_detector.iou_threshold =
        static_cast<float>(node.get_parameter("od_iou_threshold").as_double());

    node.declare_parameter("od_publish_ground_cloud", false);
    s.obstacle_detector.publish_ground_cloud =
        node.get_parameter("od_publish_ground_cloud").as_bool();

    node.declare_parameter("od_publish_cluster_cloud", false);
    s.obstacle_detector.publish_cluster_cloud =
        node.get_parameter("od_publish_cluster_cloud").as_bool();

    node.declare_parameter("lidar_reacquire_max_time", 0.5);
    s.tracking.lidar_reacquire_max_time =
        node.get_parameter("lidar_reacquire_max_time").as_double();

    node.declare_parameter("lidar_reacquire_max_dist", 3.0);
    s.tracking.lidar_reacquire_max_dist =
        node.get_parameter("lidar_reacquire_max_dist").as_double();

    node.declare_parameter("filters_manual_roi_size",
                           std::vector<double>{1.0, 1.0, 1.0});
    s.filter.manual_roi_size = Eigen::Vector3d(
        node.get_parameter("filters_manual_roi_size").as_double_array()[0],
        node.get_parameter("filters_manual_roi_size").as_double_array()[1],
        node.get_parameter("filters_manual_roi_size").as_double_array()[2]);

    node.declare_parameter("recovery_no_movement_enable", true);
    s.recovery.no_movement_enabled =
        node.get_parameter("recovery_no_movement_enable").as_bool();

    node.declare_parameter("recovery_no_movement_threshold", 0.2);
    s.recovery.no_movement_threshold =
        node.get_parameter("recovery_no_movement_threshold").as_double();

    node.declare_parameter("recovery_no_movement_window_time", 5.0);
    s.recovery.no_movement_window_time =
        node.get_parameter("recovery_no_movement_window_time").as_double();

    node.declare_parameter(
        "recovery_no_movement_check_in_states",
        std::vector<std::string>{"lidar_only", "camera_only", "full"});
    for (const auto& state_name :
         node.get_parameter("recovery_no_movement_check_in_states")
             .as_string_array()) {
        s.recovery.no_movement_check_in_states.push_back(
            ToTrackerState(state_name));
    }

    node.declare_parameter("recovery_no_movement_backoff_time", 5.0);
    s.recovery.no_movement_backoff_time =
        node.get_parameter("recovery_no_movement_backoff_time").as_double();

    node.declare_parameter("recovery_timeout_enable", true);
    s.recovery.timeout_enabled =
        node.get_parameter("recovery_timeout_enable").as_bool();

    node.declare_parameter("recovery_timeout_allow_never_tracked", false);
    s.recovery.timeout_allow_never_tracked =
        node.get_parameter("recovery_timeout_allow_never_tracked").as_bool();

    node.declare_parameter("recovery_timeout_max_attempts", 3);
    s.recovery.timeout_max_attempts = static_cast<int>(
        node.get_parameter("recovery_timeout_max_attempts").as_int());

    node.declare_parameter("recovery_uncertainty_enable", true);
    s.recovery.uncertainty_enabled =
        node.get_parameter("recovery_uncertainty_enable").as_bool();

    node.declare_parameter("recovery_uncertainty_threshold", 10.0);
    s.recovery.uncertainty_threshold =
        node.get_parameter("recovery_uncertainty_threshold").as_double();

    node.declare_parameter("recovery_uncertainty_window_time", 5.0);
    s.recovery.uncertainty_window_time =
        node.get_parameter("recovery_uncertainty_window_time").as_double();

    return s;
}

bool ObjectTrackerSettings::UpdateFromParameters(
    const std::vector<rclcpp::Parameter>& parameters) {
    bool changed = false;
    for (const auto& parameter : parameters) {
        if (parameter.get_name() == "camera_frame") {
            // Note: live updates do not re-apply "frame_prefix" (parity with
            // the original node behavior).
            frames.camera_frame = parameter.as_string();
        } else if (parameter.get_name() == "world_frame") {
            frames.world_frame = parameter.as_string();
        } else if (parameter.get_name() == "filters_roi_scale_factor") {
            filter.roi_scale_factor = parameter.as_double();
        } else if (parameter.get_name() == "filters_kalman_rate") {
            // Note: already-constructed filters and timers are not rebuilt on
            // a live rate/variance update (parity with the original node).
            filter.estimator_rate = parameter.as_double();
        } else if (parameter.get_name() == "filters_kalman_process") {
            filter.process_variance = parameter.as_double();
        } else if (parameter.get_name() == "filters_kalman_measurement") {
            filter.measurement_variance = parameter.as_double();
        } else if (parameter.get_name() == "tracker_timeout") {
            tracking.target_timeout = parameter.as_double();
        } else if (parameter.get_name() == "sync_enable") {
            sync.enabled = parameter.as_bool();
        } else if (parameter.get_name() == "sync_use_callback") {
            sync.use_callback_time = parameter.as_bool();
        } else if (parameter.get_name() == "sync_detection") {
            sync.max_detection_skew = parameter.as_double();
        } else if (parameter.get_name() == "heading_min_speed") {
            tracking.heading_min_speed = parameter.as_double();
        } else if (parameter.get_name() == "heading_resume_speed") {
            tracking.heading_resume_speed = parameter.as_double();
        } else if (parameter.get_name() == "filters_use_manual_roi") {
            filter.use_manual_roi = parameter.as_bool();
        } else if (parameter.get_name() == "tracker_toi_regex") {
            target_selection.toi_regex = parameter.as_string();
        } else {
            continue;
        }
        changed = true;
    }
    return changed;
}

}  // namespace perception
}  // namespace avt_341
