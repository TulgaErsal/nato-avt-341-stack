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

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.hpp>

#ifdef GTE_ROS_HUMBLE
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <avt_341/perception/filtering/cv_filter.hpp>
#include <avt_341/perception/tracking/exceptions.hpp>
#include <avt_341/perception/tracking/pixel_coordinates.hpp>

namespace avt_341 {
namespace perception {

class ObjectTrackingNode : public rclcpp::Node {
  public:
    ObjectTrackingNode();

  protected:
    /**
     * @brief Declare and retrieve the node parameters.
     */
    void GetParameters();

    /**
     * @brief Create the node subscriptions.
     */
    void CreateSubscriptions();

    /**
     * @brief Create the node timers.
     */
    void CreateTimers();

    /**
     * @brief Create the node publishers.
     */
    void CreatePublishers();

  private:
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    ToPCLCloud(sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message);

    /**
     * @brief Remove points with NaN values from a PCL XYZ point cloud.
     *
     * @param point_cloud PCL XYZ point cloud.
     */
    void RemoveNaNPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

    // Image
    // -----

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
    bool has_camera_info_;

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
    rclcpp::Time last_detection_time_;

    vision_msgs::msg::Detection2DArray detections_message_;

    double max_detection_skew_;

    /** @brief Class ID of the target.
     * @remark Despite its string representation meant for easy comparison with
     * ROS vision_msgs/ObjectHypothesisWithPose messages, this is a numerical
     * ID. */
    std::string target_class_;

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
        detections_subscription_;

    // Input cloud downsampling
    // ------------------------
    double leaf_size_;

    void DownsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

    // Camera field of view point projection
    // -------------------------------------

    /** @brief Frame ID of the camera optical frame. */
    std::string camera_frame_;

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
                               const int height,
                               const int width);

    /** @brief Whether or not to publish the camera field of view segmented
     * point cloud. */
    bool publish_fov_cloud_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        fov_cloud_publisher_;

    void FindPointsInROI(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                         const std::vector<PixelCoordinates>& coordinates,
                         const unsigned int x_min,
                         const unsigned int x_max,
                         const unsigned int y_min,
                         const unsigned int y_max);

    bool publish_roi_cloud_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        roi_cloud_publisher_;
    // -------------------------------------

    // Passthrough filtering
    // ---------------------
    pcl::PassThrough<pcl::PointXYZ> passthrough_filter_;

    double passthrough_distance_min_;

    double passthrough_distance_max_;

    void LimitSensorDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

    // Euclidean clustering
    // --------------------

    const std::pair<const pcl::PointCloud<pcl::PointXYZ>::Ptr,
                    const pcl::PointXYZ>
    ExtractEuclideanClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

    bool publish_cluster_cloud_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        cluster_publisher_;

    double clustering_tolerance_;

    int cluster_size_min_;

    int cluster_size_max_;

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

    // Object state estimation
    // -----------------------

    void EstimatorTimerCallback();

    std::shared_ptr<avt_341::perception::filtering::CVFilter<3>> filter_;

    bool has_new_measurement_ = false;

    double estimator_rate_;

    double filter_process_variance_;

    double filter_measurement_variance_;

    rclcpp::TimerBase::SharedPtr estimator_timer_;

    double target_timeout_;

    // -----------------------

    // Utilities
    // ---------
    void PublishPointCloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
        const rclcpp::Time& stamp,
        const std::string& frame_id,
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher);

    bool sync_messages_;

    void Initialize();

    // Voxel grid downsampling
    // -----------------------

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter_;

    bool is_ready_to_track_ = false;

    void PublishImage();

    void GetOrientedBoundingBox(
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> point_cloud,
        pcl::PointXYZ& bounding_box_min,
        pcl::PointXYZ& bounding_box_max,
        pcl::PointXYZ& bounding_box_centroid,
        Eigen::Matrix3f& bounding_box_rotation);

    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> moi_estimation_;

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

    void PublishPose();

    // Odometry publisher
    // -------------------

    bool publish_odometry_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

    void PublishOdometry();

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

    rclcpp::Time last_detection_callback_time_;

    bool is_tracking_ = false;

    Eigen::Vector3d bounding_box_size_;

    Eigen::Vector3d bounding_box_centroid_;

    bool use_filtered_pose_;

    bool use_filtered_odometry_;

    bool use_filtered_detection_;

    Eigen::Vector3d bounding_box_centroid_filtered_;

    Eigen::Quaterniond bounding_box_orientation_;
};

} // namespace perception
} // namespace avt_341