/**
* @brief     Grouped settings for the camera/LiDAR sensor fusion object
             tracker. One ObjectTrackerSettings instance is parsed from the
             ROS parameters by the owning node and shared by every replicated
             per-target ObjectTracker instance. Parameter defaults are defined
             by the declare_parameter calls in ObjectTrackerSettings::Load.
*/

#ifndef AVT_341_TRACKER_PARAMS_H
#define AVT_341_TRACKER_PARAMS_H

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <avt_341/perception/tracking/tracker_dto.hpp>

namespace avt_341 {
namespace perception {

/** @brief TF frame IDs. "frame_prefix" is applied to camera_frame (and to
 *         ObstacleDetectorSettings::robot_base_link) at parse time and is
 *         never stored on its own. */
struct FrameSettings {
    /** @brief Frame ID of the camera optical frame ("camera_frame"). */
    std::string camera_frame;
    /** @brief Frame ID of the world (fixed) frame ("world_frame"). */
    std::string world_frame;
    /** @brief Fallback child frame ID for Odometry messages
     *         ("odometry_child_frame"). Trackers derive "<target_ns>/odom"
     *         from their target class instead. */
    std::string odometry_child_frame;
};

/** @brief IMM / Kalman estimator settings. */
struct FilterSettings {
    /** @brief Estimator timer rate [Hz] ("filters_kalman_rate"). */
    double estimator_rate;
    /** @brief Process noise variance ("filters_kalman_process"). */
    double process_variance;
    /** @brief Measurement noise variance ("filters_kalman_measurement"). */
    double measurement_variance;
    /** @brief IMM: initial probability for the CV model
     *         ("filters_imm_cv_init_prob"). */
    double imm_cv_init_prob;
    /** @brief IMM: initial probability for the CTR model
     *         ("filters_imm_ctr_init_prob"). */
    double imm_ctr_init_prob;
    /** @brief IMM: initial probability for the NM model
     *         ("filters_imm_nm_init_prob"). */
    double imm_nm_init_prob;
    /** @brief IMM: diagonal entry of the Markov model-transition matrix
     *         ("filters_imm_persistence_prob"). */
    double imm_persistence_prob;
    /** @brief ROI scale factor ("filters_roi_scale_factor"). */
    double roi_scale_factor;
    /** @brief Use the manually-sized ROI ("filters_use_manual_roi"). */
    bool use_manual_roi;
    /** @brief Manual ROI bounding box size [m] ("filters_manual_roi_size"). */
    Eigen::Vector3d manual_roi_size;
};

/** @brief Camera range-from-bounding-box measurement model. */
struct CameraModelSettings {
    /** @brief Known height of the target vehicle [m], used for camera-based
     *         range estimation from bounding-box pixel height
     *         ("camera_target_height"). */
    double target_height;
    /** @brief Standard deviation of the bounding-box pixel measurement [px],
     *         used to propagate pixel uncertainty into 3D position covariance
     *         ("camera_bbox_pixel_sigma"). */
    double bbox_pixel_sigma;
};

/** @brief Detection/measurement synchronization. */
struct SyncSettings {
    /** @brief Whether to check for stale detections ("sync_enable"). */
    bool enabled;
    /** @brief Use the callback wall time instead of message stamps
     *         ("sync_use_callback"). */
    bool use_callback_time;
    /** @brief Maximum allowed detection age [s] ("sync_detection"). */
    double max_detection_skew;
};

/** @brief Output enable flags. */
struct PublishSettings {
    /** @brief Publish raw/tracked odometry ("publish_odometry"). */
    bool odometry;
    /** @brief Publish the fused Detection3D ("publish_detection"). */
    bool detection_3d;
    /** @brief Publish the debug detection image ("publish_image"). */
    bool image;
};

/** @brief Tracking loop, timeout, association, re-acquisition and heading
 *         hold settings. */
struct TrackingSettings {
    /** @brief Tracking timer rate [Hz] ("tracking_rate"). */
    double tracking_rate;
    /** @brief Tracker info publishing rate [Hz] ("info_rate"). */
    double info_rate;
    /** @brief Tracking timeout [s] ("tracker_timeout"). */
    double target_timeout;
    /** @brief Maximum pixel distance (as a multiple of the detection bbox
     *         half-diagonal) for associating an obstacle marker with a camera
     *         detection ("obstacle_association_max_dist"). */
    double obstacle_association_max_dist;
    /** @brief Maximum elapsed time [s] since the tracked obstacle was last
     *         seen during which a nearby marker is accepted as a
     *         re-acquisition of the same obstacle
     *         ("lidar_reacquire_max_time"). */
    double lidar_reacquire_max_time;
    /** @brief Maximum world-frame distance [m] for a candidate marker to be
     *         accepted as a re-acquisition ("lidar_reacquire_max_dist"). */
    double lidar_reacquire_max_dist;
    /** @brief Speed [m/s] below which the published heading is held
     *         ("heading_min_speed"). */
    double heading_min_speed = 0.5;
    /** @brief Speed [m/s] above which the heading hold is released
     *         ("heading_resume_speed"). */
    double heading_resume_speed = 1.0;
};

/** @brief Target selection / autostart settings. */
struct TargetSelectionSettings {
    /** @brief Whether to start tracking the autostart target classes
     *         automatically ("tracker_autostart"). */
    bool use_autostart;
    /** @brief Whether to listen to targets from the mission manager. Note
     *         that this disables the target selection ROS service
     *         ("tracker_use_mission_manager"). */
    bool use_mission_manager;
    /** @brief Target classes tracked from startup
     *         ("tracker_target_classes"). */
    std::vector<std::string> autostart_target_classes;
    /** @brief List of vehicle IDs in our formation. Contact updates are
     *         suppressed for these ("formation_vehicle_ids"). */
    std::vector<std::string> formation_vehicle_ids;

    /** @brief Whether to enable tracking of multiple vehicles or only support
     *         tracking a single vehicle at once ("tracker_use_multi_tracking") */
    bool use_multi_tracking = false;

    /** @brief Target ids matching this regex are targets of interest (TOI);
     *         only their trackers publish target contacts
     *         ("tracker_toi_regex"). */
    std::string toi_regex;

    /** @brief If true, will track generic targets not part of formation
     *         vehicles or toi objects ("tracker_allow_generic"). */
    bool allow_generic = true;
};

/** @brief Recovery-behavior trigger settings ("recovery_*" parameters). */
struct RecoverySettings {
    /** @brief Enable the no-movement lost check
     *         ("recovery_no_movement_enable"). */
    bool no_movement_enabled = true;
    /** @brief Windowed-mean speed [m/s] under which no movement is suspected
     *         ("recovery_no_movement_threshold"). */
    double no_movement_threshold;
    /** @brief Averaging window [s] ("recovery_no_movement_window_time"). */
    double no_movement_window_time;
    /** @brief States in which the no-movement check runs; parsed from state
     *         names ("recovery_no_movement_check_in_states"). */
    std::vector<TrackerState> no_movement_check_in_states;
    /** @brief Wait [s] between no-movement confirmations
     *         ("recovery_no_movement_backoff_time"). */
    double no_movement_backoff_time;
    /** @brief Enable the measurement-timeout lost check: measurements
     *         starved after active tracking ("recovery_timeout_enable"). */
    bool timeout_enabled = true;
    /** @brief Also fire the timeout check when the target was never acquired
     *         at all, e.g. vehicles tracking each other from far away
     *         ("recovery_timeout_allow_never_tracked"). */
    bool timeout_allow_never_tracked = false;
    /** @brief Timeout-triggered recoveries without re-acquisition before
     *         giving up ("recovery_timeout_max_attempts"). */
    int timeout_max_attempts = 3;
    /** @brief Enable the uncertainty lost check
     *         ("recovery_uncertainty_enable"). */
    bool uncertainty_enabled = true;
    /** @brief Windowed-mean std dev [m] along the axis of largest x/y
     *         variance above which the tracker is lost
     *         ("recovery_uncertainty_threshold"). */
    double uncertainty_threshold;
    /** @brief Averaging window [s] ("recovery_uncertainty_window_time"). */
    double uncertainty_window_time;
};

/** @brief Integrated LiDAR obstacle detector settings (node-owned pipeline,
 *         "od_*" parameters). */
struct ObstacleDetectorSettings {
    /** @brief Frame ID of the robot base link ("od_robot_base_link",
     *         "frame_prefix" applied at parse time). */
    std::string robot_base_link;
    /** @brief Use PCA-aligned (true) or axis-aligned (false) bounding boxes
     *         ("od_use_pca_box"). */
    bool use_pca_box;
    /** @brief Run the Hungarian-algorithm box tracker across frames
     *         ("od_use_tracking"). */
    bool use_tracking;
    /** @brief Voxel grid leaf size [m] ("od_voxel_grid_size"). */
    float voxel_grid_size;
    /** @brief ROI crop-box corners in the robot base link frame
     *         ("od_roi_max_{x,y,z}" / "od_roi_min_{x,y,z}"). */
    Eigen::Vector4f roi_max_point;
    Eigen::Vector4f roi_min_point;
    /** @brief Ego-vehicle body crop-box corners
     *         ("od_body_max_{x,y,z}" / "od_body_min_{x,y,z}"). */
    Eigen::Vector4f body_max_point;
    Eigen::Vector4f body_min_point;
    /** @brief Ground normal in the fixed frame ("od_ground_normal_{x,y,z}"). */
    Eigen::Vector3f ground_normal;
    /** @brief Dot-product threshold for ground classification
     *         ("od_ground_normal_threshold"). */
    float ground_normal_threshold;
    /** @brief Normal estimation / radius-outlier removal radius [m]
     *         ("od_obstacle_scale"). */
    float obstacle_scale;
    /** @brief Minimum neighbors for radius-outlier removal
     *         ("od_obstacle_min_neighbors"). */
    int obstacle_min_neighbors;
    /** @brief Euclidean cluster tolerance [m] ("od_cluster_threshold"). */
    float cluster_threshold;
    /** @brief Minimum / maximum cluster point counts
     *         ("od_cluster_min_size" / "od_cluster_max_size"). */
    int cluster_min_size;
    int cluster_max_size;
    /** @brief Displacement / IoU thresholds for the box tracker
     *         ("od_displacement_threshold" / "od_iou_threshold"). */
    float displacement_threshold;
    float iou_threshold;
    /** @brief Publish the ground-classified cloud
     *         ("od_publish_ground_cloud"). */
    bool publish_ground_cloud;
    /** @brief Publish the non-ground cluster cloud
     *         ("od_publish_cluster_cloud"). */
    bool publish_cluster_cloud;
};

/** @brief Overall object tracker settings: one instance owned by the node,
 *         copied into every per-target ObjectTracker instance. */
struct ObjectTrackerSettings {
    FrameSettings frames;
    FilterSettings filter;
    CameraModelSettings camera;
    SyncSettings sync;
    PublishSettings publish;
    TrackingSettings tracking;
    TargetSelectionSettings target_selection;
    ObstacleDetectorSettings obstacle_detector;
    RecoverySettings recovery;

    /**
     * @brief Declare every tracker parameter on @p node (with its default)
     * and read it back. Applies "frame_prefix" to frames.camera_frame and
     * obstacle_detector.robot_base_link. Call exactly once per node.
     */
    static ObjectTrackerSettings Load(rclcpp::Node& node);

    /**
     * @brief Apply the runtime dynamic parameter reconfiguration subset.
     *
     * @param parameters A vector of modified parameters.
     * @return Whether any settings field changed.
     */
    bool UpdateFromParameters(const std::vector<rclcpp::Parameter>& parameters);
};

}  // namespace perception
}  // namespace avt_341

#endif  // AVT_341_TRACKER_PARAMS_H
