/**
* @file      object_tracker.hpp
* @brief     Per-target core of the camera/LiDAR sensor fusion object
             tracker: association state machine, IMM filter, timeout and
             re-acquisition logic, and the per-target publishers. One
             instance is replicated per tracked target class and ticked by
             the owning node's timers.
*/

#ifndef AVT_341_OBJECT_TRACKER_H
#define AVT_341_OBJECT_TRACKER_H

#include <memory>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <opencv2/opencv.hpp>

#include <avt_341/core/coord_transform.hpp>
#include <avt_341/perception/filtering/imm_filter.hpp>
#include <avt_341/perception/tracking/tracker_params.hpp>
#include <avt_341/perception/tracking/tracker_dto.hpp>

namespace avt_341 {
namespace perception {

/**
 * @brief Per-target tracking core: camera/LiDAR association state machine,
 * IMM filter, timeout/re-acquisition logic, and the per-target publishers.
 *
 * The target class is immutable for the lifetime of an instance; re-targeting
 * an existing class is expressed as Reset() on the existing instance. Not
 * thread safe: instances must only be touched from the owning node's
 * callbacks (single-threaded executor).
 */
class ObjectTracker {
   public:
    ObjectTracker(rclcpp::Node* node,
                  const std::string& target_class,
                  const ObjectTrackerSettings& settings,
                  const core::CoordTransformer& coord_transformer,
		rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr target_contacts_publisher,
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr leader_odom_publisher
	);

    // Per-tick entry points (called from the owning node's timers)
    // -------------------------------------------------------------------------

    /**
     * @brief Run one tracking state-machine tick: associate the cached
     * camera detection with the obstacle detector markers, handle LiDAR-only
     * re-acquisition and camera-only fallback, and update the timeout state.
     *
     * @param context Shared sensor context for this tick.
     */
    void TrackingTick(const TrackerSensorContext& context);

    /**
     * @brief Run one estimator tick: IMM filter predict/update with chi2
     * gating, and publish the odometry/detection outputs.
     */
    void EstimatorTick();

    // Detection ingestion (called from the owning node's detections callback)
    // -------------------------------------------------------------------------

    /**
     * @brief Cache one camera detection matching this tracker's target class.
     *
     * Rejects detections whose bounding box touches an image edge (a clipped
     * bbox produces a biased centroid estimate). The rejection requires
     * camera intrinsics and is skipped while @p camera_info is null.
     *
     * @param detection The matching vision_msgs/Detection2D.
     * @param header_stamp Stamp of the enclosing Detection2DArray header.
     * @param camera_info Latest camera intrinsics (may be null).
     */
    void IngestDetection(
        const vision_msgs::msg::Detection2D& detection,
        const rclcpp::Time& header_stamp,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info);

    /** @brief No detection matching this tracker's target class was found in
     *         the current frame. */
    void MarkDetectionMiss();

    // Lifecycle
    // -------------------------------------------------------------------------

    /**
     * @brief Full state reset: filter zeroed, flags and counters cleared,
     * state set to INACTIVE. Publishers are not recreated (the target class
     * is immutable, so the topic names cannot change).
     */
    void Reset();

    /** @brief Replace the shared settings (dynamic parameter propagation). */
    void UpdateSettings(const ObjectTrackerSettings& settings);

    // Accessors
    // -------------------------------------------------------------------------

    TrackerState GetTrackerState() const { return state_; }

    const std::string& GetTargetClass() const { return target_class_; }

   private:
    void CreatePerTargetPublishers();

    void CheckTargetTimeout();

    // JN addition
    /** @brief Get coordinates from camera bounding box when the LiDAR
     *         centroid is unavailable. */
    void CameraCentroidEstimate();

    // JN addition for camera detection only tracking
    /** @brief Estimate range from bbox detection using pixel height vs
     *         vehicle height and return point measurement of the bbox center
     *         in 3D (right-down-front frame). */
    Eigen::Vector3d ConvertBBoxCoordinatesToPoseCentroid_rdf(
        const vision_msgs::msg::Detection2D& detections_message,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info_message);
   
    // JN addition for better pose measurement
     /** @brief Improve detection by fitting planes to point cloude end and flank given current yaw 
     *         Input: pcl cluster object_cluster from obstacle detector, current yaw is read from global variable (last_reliable_yaw_)
     *        return covariance matirix and improved_centroid and improved_yaw */
    Eigen::Matrix3d ImprovePoseMeasurement(pcl::PointCloud<pcl::PointXYZ>::Ptr object_cluster,
        Eigen::Vector3d measured_centroid, const std::string& source_frame,
        const std::string& target_frame, double current_yaw, double platform_yaw, double& improved_yaw);
    double current_yaw_; //keep track of yaw from ImprovePoseMeasurement
    double current_yaw_info_; //keep track of yaw information for fusion using informationfilter
    double yaw_info_;
    pcl::SACSegmentation<pcl::PointXYZ> sac_segmentation_;
    void UpdateHeadingHold();

    void PublishOdometry();

    void PublishDetection3D();

    void PublishTargetContact();

    void MaybePublishContactUpdate();

    // Wiring (non-owning except the publishers)
    // -------------------------------------------------------------------------

    rclcpp::Node* node_;

    rclcpp::Logger logger_;

    std::string target_class_;

    /** @brief Sanitized topic/frame namespace for this target. */
    std::string target_ns_;

    /** @brief Child frame ID for the Odometry messages of this target. */
    std::string odometry_child_frame_;

    ObjectTrackerSettings settings_;

    /** @brief Shared coordinate transformer owned by the node. */
    const core::CoordTransformer& coord_transformer_;
	// TODO: Move to node
    /** @brief Shared target contacts publisher owned by the node. */
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr target_contacts_publisher_;
	// TODO: Move to node
	/** Single common odometry topic for tracked lead vehicle */
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr leader_odom_publisher_;
    /** @brief Latest camera intrinsics, cached on each tracking tick. */
    sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;

    // Per-target publishers (created once in the constructor)
    // -------------------------------------------------------------------------

    rclcpp::Publisher<vision_msgs::msg::Detection3D>::SharedPtr
        detection_publisher_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
        tracked_target_odometry_publisher_;

    // Object state estimation (IMM: CV + CTR + NM)
    // -------------------------------------------------------------------------

    std::shared_ptr<avt_341::perception::filtering::IMMFilter> filter_;

    bool filter_initialized_ = false;

    bool has_new_measurement_ = false;

    /** @brief True once the first successful LiDAR measurement has been
     *         processed. Camera-only updates are suppressed until then to
     *         prevent noisy range estimates from drifting the filter before
     *         LiDAR confirms the target position. */
    bool has_had_first_lidar_measurement_ = false;

    TrackerState state_ = TrackerState::UNINITIALIZED;

    // Detection cache
    // -------------------------------------------------------------------------

    vision_msgs::msg::Detection2D detections_message_;

    double detection_score_ = 0.0;

    /** @brief Whether or not there is a valid detection containing the target
     *         within the synchronization window. */
    bool has_detection_ = false;

    /** @brief Whether or not the first valid detection containing the target
     *         has been received since the tracker reset. */
    bool has_first_detection_ = false;

    /** @brief Time stamp of the last valid detection message containing the
     *         target. */
    rclcpp::Time last_valid_detection_time_;

    rclcpp::Time last_valid_detection_callback_time_;

    // Tracker timeout handling
    // -------------------------------------------------------------------------

    /** @brief Indicates whether or not a valid and recent tracked target
     *         centroid is available. */
    bool has_tracked_target_ = false;

    /** @brief Time stamp of the last valid measurement from any source
     *         (camera or LiDAR). CheckTargetTimeout() fires after
     *         target_timeout seconds without an update from either sensor. */
    rclcpp::Time last_valid_target_time_;

    // LiDAR obstacle association and re-acquisition
    // -------------------------------------------------------------------------

    /** @brief ID of the obstacle detector marker associated with the tracked
     *         target. Set to -1 when no association has been established. */
    int tracked_obstacle_id_ = -1;

    /** @brief Last known world-frame position of the tracked obstacle. Used
     *         to re-acquire the obstacle if the LiDAR briefly loses it and
     *         reassigns it a new ID. */
    Eigen::Vector3d last_lidar_world_pos_ = Eigen::Vector3d::Zero();

    /** @brief Time at which the tracked obstacle was last successfully
     *         matched in a LiDAR tracking cycle. */
    rclcpp::Time last_lidar_seen_time_;

    // Heading hold
    // -------------------------------------------------------------------------

    double last_reliable_yaw_ = 0.0;

    bool heading_held_ = false;

    // Target contacts (encircle trigger)
    // -------------------------------------------------------------------------

    bool encircle_triggered_ = false;

    int contact_update_counter_ = 0;

    static constexpr int contact_update_interval_ticks_ = 10;

    // Measurement state
    // -------------------------------------------------------------------------

    /** @brief Camera measurement covariance in the right-down-forward frame,
     *         computed each tick from bbox pixel uncertainty and camera
     *         intrinsics. */
    Eigen::Matrix3d R_rdf_ = Eigen::Matrix3d::Identity();

    Eigen::Vector3d bounding_box_centroid_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d bounding_box_centroid_global_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d bounding_box_centroid_filtered_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d bounding_box_size_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d object_size_ = Eigen::Vector3d::Zero();

    Eigen::Quaterniond bounding_box_orientation_ = Eigen::Quaterniond::Identity();
};

}  // namespace perception
}  // namespace avt_341

#endif  // AVT_341_OBJECT_TRACKER_H
