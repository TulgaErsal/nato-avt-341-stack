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
* @brief     Header file for the camera/LiDAR sensor fusion object tracker rclcpp ROS node.
* @copyright MIT License

             NATO AVT-341 Autonomy Stack: Autonomous Navigation Stack for Ground Vehicles
             Copyright (c) 2024 Dario Sirangelo (dsi@aarhusrobotics.com).

             NOTE: The above copyright only applies to the contents of this file. The source code contained in this file
             is a direct port from the GitHub repository aarhus-robotics/navi, released by the copyright holder under
             the MIT license.

             Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
             associated documentation files (the "Software"), to deal in the Software without restriction, including
             without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
             copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
             following conditions:

             The above copyright notice and this permission notice shall be included in all copies or substantial
             portions of the Software.

             THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
             LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO
             EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
             IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
             THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <avt_341/perception/filtering/cv_filter.hpp>
#include <avt_341/perception/tracking/pixel_coordinates.hpp>

namespace avt_341 {
namespace perception {

class ObjectTrackingNode : public rclcpp::Node {
  public:
    ObjectTrackingNode();

  protected:
    void GetParameters();

    void CreateSubscriptions();

    void CreateTimers();

    void CreatePublishers();

  private:
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detections_publisher_;

    // Input cloud downsampling
    // ------------------------
    double leaf_size_;

    void DownsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, const float& leaf_size);

    // Input point cloud processing
    // ----------------------------

    /** @brief Shared pointer to the point cloud subscription. */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;

    void PointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ToPCLCloud(sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message);

    rclcpp::Time cloud_time_;

    void DensifyCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);

    // ----------------------------

    // Camera and object detection
    // ---------------------------

    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr image_message);

    void DetectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr detections_message);

    void CameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_message);

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_subscription_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

    /** @brief Shared pointer to the camera info subscription. */
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;

    /** @brief */
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_message_;

    bool has_camera_info_;

    /** @brief Constant shared pointer to the latest received image in a
     * cv_bridge wrapper. */
    cv_bridge::CvImageConstPtr latest_image_;

    bool has_image_ = false;
    bool has_detection_ = false;

    vision_msgs::msg::Detection2DArray detections_message_;

    rclcpp::Time last_detection_time_;

    double max_detection_skew_;

    /** @brief Shared pointer to the object detections overlay image publisher.
     */
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

    // Camera field of view point projection
    // -------------------------------------

    /** @brief Frame ID of the camera optical frame. */
    std::string camera_frame_;

    /** @brief Shared pointer to the transform listener. */
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

    /** @brief Unique pointer to the transform buffer. */
    std::unique_ptr<tf2_ros::Buffer> transform_buffer_;

    sensor_msgs::msg::PointCloud2 TransformPointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr point_cloud_message,
                                                      const std::string target_frame);

    void TransformPointCloud(sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message,
                             const std::string target_frame);

    PixelCoordinates ConvertPointToPixelCoordinates(pcl::PointXYZ& point,
                                                    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_message);

    std::vector<PixelCoordinates>
    ConvertPointCloudToPixelCoordinates(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                                        const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_message);

    pcl::PointCloud<pcl::PointXYZ>::Ptr FindPointsInCameraFOV(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                                                              const std::vector<PixelCoordinates>& coordinates,
                                                              const int height,
                                                              const int width);

    pcl::PointCloud<pcl::PointXYZ>::Ptr FindPointsInROI(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                                                        const std::vector<PixelCoordinates>& coordinates,
                                                        const unsigned int x_min,
                                                        const unsigned int x_max,
                                                        const unsigned int y_min,
                                                        const unsigned int y_max);

    /** @brief Whether or not to publish the camera field of view segmented point cloud. */
    bool publish_fov_cloud_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fov_cloud_publisher_;
    // -------------------------------------

    // Passthrough filtering
    // ---------------------
    void LimitSensorDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, const float distance);

    // Euclidean clustering
    // --------------------

    float cluster_tolerance_;

    int minimum_cluster_size_;

    int maximum_cluster_size_;

    const std::pair<const pcl::PointCloud<pcl::PointXYZ>::Ptr, const pcl::PointXYZ>
    ExtractEuclideanClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                             const float tolerance,
                             const int cluster_size_min,
                             const int cluster_size_max);

    bool publish_cluster_cloud_;

    Eigen::Quaternion<float> estimated_orientation_;

    rclcpp::Publisher<vision_msgs::msg::Detection3D>::SharedPtr detection_publisher_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_publisher_;

    // Ground plane segmentation
    // -------------------------

    void SegmentGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane,
                            const int& max_iterations,
                            const float& threshold);

    /** @brief Whether or not to publish the point cloud of the segmented ground
     * plane. */
    bool publish_ground_cloud_;

    /** @brief Shared pointer to the segmented ground plane point cloud
     * publisher */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_cloud_publisher_;

    /** @brief RANSAC fit distance threshold. */
    float sac_segmentation_threshold_;

    /** @brief Maximum number of RANSAC fit iterations. */
    float sac_segmentation_max_iterations_;
    // -------------------------

    // Object state estimation
    // -----------------------

    void EstimatorTimerCallback();

    std::shared_ptr<avt_341::perception::filtering::CVFilter<3>> filter_;

    bool has_new_measurement_ = false;

    double estimator_rate_;

    double filter_process_variance_;

    double filter_measurement_variance_;

    rclcpp::TimerBase::SharedPtr estimator_timer_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

    // -----------------------

    // Utilities
    // ---------
    void PublishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                           rclcpp::Time stamp,
                           std::string frame_id,
                           rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher);
    bool sync_messages_;

    int number_of_detections_ = 0;

    pcl::PointXYZ centroid_;

    // ---------
};

} // namespace perception
} // namespace avt_341