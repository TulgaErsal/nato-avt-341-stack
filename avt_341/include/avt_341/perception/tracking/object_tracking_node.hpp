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

* @file      object_tracking_node.hpp
* @author    Dario Sirangelo (dsi@aarhusrobotics.com)
* @brief     Header file for the camera/LiDAR sensor fusion object tracker
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

#pragma once

#include <tuple>
#include <utility>

#ifdef GTE_ROS_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <tf2/convert.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#ifdef GTE_ROS_HUMBLE
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <avt_341/perception/filtering/imm_filter.hpp>
#include <avt_341/perception/tracking/exceptions.hpp>
#include <avt_341/perception/tracking/pixel_coordinates.hpp>
#include <avt_341_msgs/msg/mission_task_status.hpp>
#include <avt_341_msgs/msg/tracker_info.hpp>
#include <avt_341_msgs/srv/set_target.hpp>

namespace avt_341 {
namespace perception {

/**
         * @brief State of the tracker.
         *
         * - UNININITIALIZED: The tracker is launching and not yet ready to
                              track.
         * - INACTIVE: The tracker has yet to receive an initial valid camera
                       detection to initialize the tracking.
         * - NO_DETECTION: No camera or LiDAR detection available: the
                           estimation filter is publishing a predicted target
                           odometry.
         * - LIDAR_ONLY_TRACKING: No camera detection is available: the target
                                  is being tracked on the LiDAR region of
                                  interest through state estimation.
         * - FULL_TRACKING: Both camera and LiDAR detections are available: the
                            target is being tracked in the region of interest
                            defined by the camera.
         */
enum TrackerState {
    UNINITIALIZED = 0,
    INACTIVE = 1,
    NO_DETECTION = 2,
    LIDAR_ONLY_TRACKING = 3,
    FULL_TRACKING = 4,
    CAMERA_ONLY_TRACKING = 5
};

class BoundingBox2D {
   public:
    BoundingBox2D() {}

    BoundingBox2D(const unsigned int& center_x, const unsigned int& center_y,
                  const unsigned int& size_x, const unsigned int& size_y);

   private:
    unsigned int center_x_;
    unsigned int center_y_;
    unsigned int size_x_;
    unsigned int size_y_;
};

class Cluster {
   public:
    Cluster() {}

    Cluster(const Eigen::Vector3d centroid, const unsigned int& size);

   private:
    Eigen::Vector3d centroid_;
    unsigned int size_;
};

class ObjectTrackingNode : public rclcpp::Node {
   public:
    ObjectTrackingNode();

   private:
    // ROS node interface
    // -------------------------------------------------------------------------
    /**
     * @brief Declare and retrieve the ROS node parameters.
     */
    void GetParameters();

    /**
     * @brief Create the ROS node subscriptions.
     */
    void CreateSubscriptions();

    /**
     * @brief Create the ROS node timers.
     */
    void CreateTimers();

    /**
     * @brief Create the ROS node publishers.
     */
    void CreatePublishers();

    /**
     * @brief Create the ROS node services.
     */
    void CreateServices();

    // Runtime dynamic parameter reconfiguration.
    // -------------------------------------------------------------------------
    /** @brief Callback handle for runtime dynamic parameter reconfiguration. */
    OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

    /**
     * @brief Callback for the runtime dynamic parameter reconfiguration.
     *
     * @param parameters A vector of modified parameters.
     * @return rcl_interfaces::msg::SetParametersResult The outcome of the
     * runtime dynamic parameter reconfiguration operation.
     */
    rcl_interfaces::msg::SetParametersResult SetParametersCallback(
        const std::vector<rclcpp::Parameter>& parameters);
    // -------------------------------------------------------------------------

    // Input point cloud processing
    // ----------------------------

    /** @brief Point cloud subscription. */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        point_cloud_subscription_;

    /**
     * @brief Point cloud subscription callback.
     *
     * @param point_cloud_message ROS sensor_msgs/PointCloud2 message.
     */
    void PointCloudCallback(
        sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message);

    /**
     * @brief Convert a ROS sensor_msgs/PointCloud2 message to PCL XYZ point
     * cloud.
     *
     * @param point_cloud_message ROS sensor_msgs/PointCloud2 message.
     * @return pcl::PointCloud<pcl::PointXYZ>::Ptr PCL XYZ point cloud.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr ToPCLCloud(
        sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message);

    /**
     * @brief Remove points with NaN values from a PCL XYZ point cloud.
     *
     * @param point_cloud PCL XYZ point cloud.
     */
    void RemoveNaNPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

    /** @brief Camera info subscription. */
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        camera_info_subscription_;

    /**
     * @brief Camera info subscription callback.
     *
     * @param camera_info_message ROS sensor_msgs/CameraInfo message.
     */
    void CameraInfoCallback(
        const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_message);

    /** @brief Whether or not camera info has been received. */
    bool has_camera_info_ = false;

    /** @brief Latest received camera info message. */
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_message_;

    // Detection
    // ---------

    void DetectionsCallback(
        const vision_msgs::msg::Detection2DArray::SharedPtr detections_message);

    /** @brief Whether or not the first valid detection containing the target
     * has been received since the tracker reset. */
    bool has_first_detection_ = false;

    /** @brief Whether or not there is a valid detection containing the target
     * within the synchronization window. */
    bool has_detection_ = false;

    double detection_score_;

    /** @brief Time stamp of the last valid detection message containing the
     * target. */
    rclcpp::Time last_valid_detection_time_;

    vision_msgs::msg::Detection2D detections_message_;

    double max_detection_skew_;

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
        detections_subscription_;

    // Camera field of view point projection
    // -------------------------------------

    /** @brief Frame ID of the camera optical frame. */
    std::string camera_frame_;

    bool centroid_in_cloud_frame_ = false;

    /** @brief Child frame ID for the Odometry message. */
    std::string odometry_child_frame_;

    /** @brief Shared pointer to the transform listener. */
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

    /** @brief Unique pointer to the transform buffer. */
    std::unique_ptr<tf2_ros::Buffer> transform_buffer_;

    geometry_msgs::msg::TransformStamped TransformPointCloud(
        sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message,
        const std::string target_frame);

    // Projection
    // ----------

    PixelCoordinates ConvertPointToPixelCoordinates(
        const pcl::PointXYZ& point,
        const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_message);

    std::vector<PixelCoordinates> ConvertPointCloudToPixelCoordinates(
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
        const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_message);

    // Camera FOV
    // ----------

    void FindPointsInCameraFOV(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                               const std::vector<PixelCoordinates>& coordinates,
                               const int height, const int width);

    /** @brief Whether or not to publish the camera field of view segmented
     * point cloud. */
    bool publish_fov_cloud_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        fov_cloud_publisher_;

    void FindPointsInROI(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                         const std::vector<PixelCoordinates>& coordinates,
                         const unsigned int x_min, const unsigned int x_max,
                         const unsigned int y_min, const unsigned int y_max);

    bool publish_roi_cloud_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        roi_cloud_publisher_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        cropbox_cloud_publisher_;

    // JN addition for camera detection only tracking
    /** @brief Estimate range from BBox detection using pixel height vs vehicle height
    * and return point measurment of BBox center in 3D. */
    Eigen::Vector3d ConvertBBoxCoordinatesToPoseCentroid_rdf(
        const vision_msgs::msg::Detection2D& detections_message,
        const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_message);

    // -------------------------------------

    // Passthrough filtering
    // ---------------------
    pcl::PassThrough<pcl::PointXYZ> passthrough_filter_;

    double passthrough_distance_min_;

    double passthrough_distance_max_;

    void LimitSensorDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                             bool symmetric);

    // Euclidean clustering
    // --------------------

    const std::pair<const pcl::PointCloud<pcl::PointXYZ>::Ptr,
                    const pcl::PointXYZ>
    ExtractEuclideanClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

    bool publish_cluster_cloud_;

    bool publish_cropbox_cloud_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        cluster_publisher_;

    double clustering_tolerance_;

    int cluster_size_min_;

    int cluster_size_max_;

    /** @brief Minimum vertical extent (m) for a cluster to be considered a
     *         valid vehicle. Rejects flat ground patches missed by RANSAC and
     *         very small returns. */
    double cluster_height_min_;

    /** @brief Maximum vertical extent (m) for a cluster to be considered a
     *         valid vehicle. Rejects large static objects such as buildings. */
    double cluster_height_max_;

    /** @brief Minimum horizontal (left-right) extent (m) of a valid cluster.
     *         In the camera optical frame this is the X axis. */
    double cluster_width_min_;

    /** @brief Maximum horizontal (left-right) extent (m) of a valid cluster. */
    double cluster_width_max_;

    /** @brief Minimum along-range extent (m) of a valid cluster.
     *         In the camera optical frame this is the Z axis. */
    double cluster_depth_min_;

    /** @brief Maximum along-range extent (m) of a valid cluster. */
    double cluster_depth_max_;

    /** @brief Reference range (m) at which filters_clustering_size_minimum
     *         applies. The minimum point count scales as 1/d^2 relative to
     *         this distance to account for LiDAR return density fall-off. */
    double cluster_distance_ref_;

    // Ground plane segmentation
    // -------------------------

    void SegmentGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane);

    /** @brief Whether or not to publish the point cloud of the segmented ground
     * plane. */
    bool publish_ground_cloud_;

    /** @brief Shared pointer to the segmented ground plane point cloud
     * publisher */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        ground_cloud_publisher_;

    // Object state estimation (IMM: CV + CTR + NM)
    // ----------------------------------------

    void EstimatorTimerCallback();

    std::shared_ptr<avt_341::perception::filtering::IMMFilter> filter_;

    bool has_new_measurement_ = false;

    /** @brief True once the first successful LiDAR cluster measurement has
     *         been processed. Camera-only updates are suppressed until then
     *         to prevent noisy range estimates from drifting the filter before
     *         LiDAR confirms the target position. */
    bool has_had_first_lidar_measurement_ = false;

    double estimator_rate_;

    double filter_process_variance_;

    double filter_measurement_variance_;

    /** @brief Known height of the target vehicle [m], used for camera-based
     *         range estimation from bounding-box pixel height. */
    double camera_target_height_;

    /** @brief Standard deviation of the bounding-box pixel measurement [px],
     *         used to propagate pixel uncertainty into 3D position covariance. */
    double camera_bbox_pixel_sigma_;

    /** @brief Camera measurement covariance in the right-down-forward frame,
     *         computed each tick from bbox pixel uncertainty and camera intrinsics. */
    Eigen::Matrix3d R_rdf_ = Eigen::Matrix3d::Identity();

    /** @brief IMM: initial probability for the CV model. */
    double imm_cv_init_prob_;

    /** @brief IMM: initial probability for the CTR model. */
    double imm_ctr_init_prob_;

    /** @brief IMM: initial probability for the NM model. */
    double imm_nm_init_prob_;

    /** @brief IMM: diagonal entry of the Markov model-transition matrix. */
    double imm_transition_prob_;

    rclcpp::TimerBase::SharedPtr estimator_timer_;

    double target_timeout_;

    // -----------------------

    // Utilities
    // ---------
    void PublishPointCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
        const rclcpp::Time& stamp, const std::string& frame_id,
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher);

    bool sync_messages_;

    void Initialize();

    // Voxel grid downsampling filter
    // -------------------------------------------------------------------------

    double leaf_size_;

    void DownsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter_;
    // -------------------------------------------------------------------------

    // PCA oriented bounding box estimation
    // -------------------------------------------------------------------------
    /** @brief Whether or not to use the centroid of the PCA oriented bounding
     * box as cluster centroid. */
    bool use_pca_centroid_;

    /** @brief Point cloud moment of inertia estimator. */
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> moi_estimation_;

    /**
     * @brief Estimate the PCA oriented bounding box for a point cloud.
     *
     * @param point_cloud The point clouds for which the PCA oriented bounding
     * box will be estimated.
     * @param bounding_box_min The minimum XYZ coordinates of the bounding box.
     * @param bounding_box_max The maximum XYZ coordinates of the bounding box.
     * @param bounding_box_centroid The XYZ coordinates of the bounding box
     * centroid.
     * @param bounding_box_rotation The rotation matrix for the bounding box.
     */
    void GetOrientedBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                                pcl::PointXYZ& bounding_box_min,
                                pcl::PointXYZ& bounding_box_max,
                                pcl::PointXYZ& bounding_box_centroid,
                                Eigen::Matrix3f& bounding_box_rotation);
    // -------------------------------------------------------------------------

    // Detection publisher
    // -------------------

    bool publish_detection_3d_;

    rclcpp::Publisher<vision_msgs::msg::Detection3D>::SharedPtr
        detection_publisher_;

    void PublishDetection3D();

    // Pose publisher
    // -------------------

    bool publish_pose_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        pose_publisher_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        pose_filtered_publisher_;

    void PublishPose();

    // Odometry publisher
    // -------------------

    bool publish_odometry_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_filtered_publisher_;

    void PublishOdometry();

    // Crop box
    // --------

    pcl::CropBox<pcl::PointXYZ> crop_box_;

    void CropRegionOfInterest();

    // SAC segmentation
    // --------------------

    pcl::SACSegmentation<pcl::PointXYZ> sac_segmentation_;

    double sac_segmentation_angle_;

    /** @brief RANSAC fit distance threshold. */
    double sac_segmentation_threshold_;

    /** @brief Maximum number of RANSAC fit iterations. */
    int sac_segmentation_max_iterations_;

    // Detection image publishing
    // --------------------------

    bool publish_image_;

    /** @brief Shared pointer to the object detections overlay image publisher.
     */
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

    bool use_callback_time_;

    rclcpp::Time last_valid_detection_callback_time_;

    bool is_tracking_ = false;

    Eigen::Vector3d bounding_box_size_;

    Eigen::Vector3d bounding_box_centroid_;

    Eigen::Vector3d bounding_box_centroid_global_;

    bool filter_initialized_;

    bool use_filtered_pose_;

    bool use_filtered_odometry_;

    bool use_filtered_detection_;

    Eigen::Vector3d bounding_box_centroid_filtered_;

    Eigen::Vector3d bounding_box_kernel_;

    Eigen::Vector3d object_size_;

    Eigen::Quaterniond bounding_box_orientation_;

    //
    TrackerState state_ = TrackerState::UNINITIALIZED;

    double roi_scale_factor_;

    // Task status
    // ------------------------------------------------------------------------

    /** @brief Mission tasks status subscription. */
    rclcpp::Subscription<avt_341_msgs::msg::MissionTaskStatus>::SharedPtr
        task_status_subscription_;
    /**
     * @brief Mission task status subscription callback.
     *
     * @param task_status_message ROS avt_341_msgs/MissionTaskStatus message.
     */
    void TaskStatusCallback(
        avt_341_msgs::msg::MissionTaskStatus::SharedPtr task_status_message);

    rclcpp::Publisher<avt_341_msgs::msg::TrackerInfo>::SharedPtr
        info_publisher_;

    void TrackerInfoCallback();

    rclcpp::TimerBase::SharedPtr tracking_timer_;

    double tracking_rate_;

    void TrackingTimerCallback();

    // Point cloud information
    // -------------------------------------------------------------------------
    /** @brief Whether or not a valid point cloud has been received since the
     * last completed tracker callback. */
    bool has_point_cloud_ = false;

    /** @brief Shared pointer to the latest received sensor_msgs/msg/PointCloud2
     * message. */
    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message_;

    /** @brief Shared pointer to the latest parsed point cloud. */
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;

    // Euclidean clustering
    // -------------------------------------------------------------------------

    std::vector<Cluster> clusters_;

    Cluster tracked_cluster_;
    // -------------------------------------------------------------------------

    // Coordinate transformations
    // -------------------------------------------------------------------------

    /**
     * @brief Transform a point cloud from its originating frame to the camera
     * frame.
     *
     * @details A valid transform must be available in the TF tree at the time
     * of invocation. Note that the transform is performed in-place, hence the
     * original point coordinates are overwritten in the process.
     *
     * @param point_cloud The point cloud to be transformed in-place.
     * @param point_cloud_message The ROS sensor_msgs/msg/PointCloud2 message
     * for the point cloud.
     */
    void TransformPointCloudToCameraFrame(
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
        sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message);

    /**
     * @brief Transform a three-dimensional point from the camera frame to the
     * fixed frame.
     *
     * @param point Point to be transformed.
     * @return Eigen::Vector3d Transformed point.
     */
    Eigen::Vector3d TransformToCoordinates(const std::string& source_frame,
                                           const std::string& target_frame,
                                           const Eigen::Vector3d& point) const;
    // -------------------------------------------------------------------------

    /** @brief The frame ID of the world (fixed) frame. */
    std::string world_frame_;
    // -------------------------------------------------------------------------

    void ProjectPointsToPixel(
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
        sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message);

    std::string ToString(TrackerState& state);

    // Target selection service
    // -------------------------------------------------------------------------
    /** @brief Whether or not to start tracking the target class automatically.
     */
    bool use_autostart_;

    /** @brief Whether or not to listen to targets from the mission manager.
     * Note that this disables the target selection ROS service. */
    bool use_mission_manager_;

    std::string target_class_;

    std::string autostart_target_class_;

    /** @brief Whether or not a target to be tracked has been selected. */
    bool has_target_selection_;

    /** @brief Service server for the target selection service. */
    rclcpp::Service<avt_341_msgs::srv::SetTarget>::SharedPtr
        set_target_service_server_;

    /**
     * @brief Target selection service callback.
     *
     * @param request Request containing the selected target ID.
     * @param response Response containing the service outcome and report
     * message.
     */
    void SetTargetServiceCallback(
        const std::shared_ptr<avt_341_msgs::srv::SetTarget::Request> request,
        std::shared_ptr<avt_341_msgs::srv::SetTarget::Response> response);
    // ---------------------------------------------------------------------- //

    // ---------------------------------------------------------------------- //
    // Tracker information
    // ---------------------------------------------------------------------- //

    /** @brief The timer for the tracker information publishing callback. */
    rclcpp::TimerBase::SharedPtr info_timer_;

    /** @brief The rate at which the tracker information message is published.
     * */
    double info_rate_;

    bool is_ready_to_track_ = false;

    // ---------------------------------------------------------------------- //
    // > Tracking image publishing
    // ---------------------------------------------------------------------- //

    /** @brief Camera image subscription. */
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
        image_subscription_;

    /**
     * @brief Camera image subscription callback.
     *
     * @param image_message ROS sensor_msgs/Image message.
     */
    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr image_message);

    /** @brief Whether or not a camera image has been received. */
    bool has_image_ = false;

    /** @brief Latest received camera image in a cv_bridge wrapper. */
    cv_bridge::CvImageConstPtr latest_image_;

    void PublishImage();

    double execution_time_ = -1.0;
    void Reset();

    void EuclideanClustering();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_;

    // JN addition
    /** @brief Get coordinates from Camera boundingbox is lidar
    *         centroid is un available. */
    void CameraCentroidEstimate();

    // ---------------------------------------------------------------------- //
    // > Tracker timeout handling
    // ---------------------------------------------------------------------- //

    /** @brief Indicates whether or not a valid and recent tracked target
     *         centroid is available. */
    bool has_tracked_target_ = false;

    /** @brief Time stamp of the last valid detection message containing the
     *         target. */
    rclcpp::Time last_valid_target_time_;

    void CheckTargetTimeout();

    // ---------------------------------------------------------------------- //
    // > LiDAR-only tracking
    // ---------------------------------------------------------------------- //

    bool use_manual_roi_size_;

    Eigen::Vector3d roi_bounding_box_3d_size_;

    BoundingBox2D roi_bounding_box_2d_;
};

}  // namespace perception
}  // namespace avt_341
