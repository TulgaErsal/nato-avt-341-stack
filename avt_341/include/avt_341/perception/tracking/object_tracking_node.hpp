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
             rclcpp ROS node. The node owns the shared sensor inputs (camera,
             LiDAR obstacle detection, TF) and replicates one ObjectTracker
             instance per tracked target class.
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

#include <map>
#include <memory>
#include <string>
#include <vector>

#ifdef GTE_ROS_JAZZY
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <tf2/convert.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

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
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <avt_341/core/coord_transform.hpp>
#include <avt_341/perception/box.hpp>
#include <avt_341/perception/lidar_obstacle_detector/ros2/lidar_obstacle_detector.hpp>
#include <avt_341/perception/tracking/object_tracker.hpp>
#include <avt_341/perception/tracking/tracker_params.hpp>
#include <avt_341_msgs/msg/mission_task_status.hpp>
#include <avt_341_msgs/msg/tracker_info.hpp>
#include <avt_341_msgs/srv/set_target.hpp>

namespace avt_341 {
namespace perception {

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
     * @brief Create the ROS node publishers (shared topics only; the
     * per-target publishers are created by each ObjectTracker instance).
     */
    void CreatePublishers();

    /**
     * @brief Create the ROS node services.
     */
    void CreateServices();

    void Initialize();

    // Settings and runtime dynamic parameter reconfiguration
    // -------------------------------------------------------------------------

    /** @brief Settings shared by the node and every tracker instance. */
    ObjectTrackerSettings settings_;

    /** @brief Callback handle for runtime dynamic parameter reconfiguration. */
    OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

    /**
     * @brief Callback for the runtime dynamic parameter reconfiguration.
     * Updated settings are propagated to every live tracker instance.
     *
     * @param parameters A vector of modified parameters.
     * @return rcl_interfaces::msg::SetParametersResult The outcome of the
     * runtime dynamic parameter reconfiguration operation.
     */
    rcl_interfaces::msg::SetParametersResult SetParametersCallback(
        const std::vector<rclcpp::Parameter>& parameters);

    // Per-target trackers
    // -------------------------------------------------------------------------

    /** @brief One tracker instance per tracked target class. */
    std::map<std::string, std::unique_ptr<ObjectTracker>> trackers_;

    /**
     * @brief Re-target or create the tracker for @p target_class: an existing
     * tracker is Reset(); a new class constructs a new tracker (which creates
     * its per-target publishers). With multi-tracking enabled existing
     * trackers are never destroyed; in single-tracking mode every existing
     * tracker is removed before the new one is created.
     */
    ObjectTracker& AddOrResetTracker(const std::string& target_class);

    /** @brief Create one tracker per autostart target class (only the first
     *         class in single-tracking mode). */
    void SpawnAutostartTrackers();

    // Coordinate transformations
    // -------------------------------------------------------------------------

    /** @brief Shared pointer to the transform listener. */
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

    /** @brief Unique pointer to the transform buffer. */
    std::unique_ptr<tf2_ros::Buffer> transform_buffer_;

    /** @brief Coordinate transformer bound to the transform buffer and the
     *         node logger, shared with the child ObjectTracker instances. */
    std::unique_ptr<core::CoordTransformer> coord_transformer_;

    // Input point cloud processing
    // -------------------------------------------------------------------------

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
    // -------------------------------------------------------------------------

    /** @brief 2D detections subscription. */
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
        detections_subscription_;

    /**
     * @brief 2D detections subscription callback. Dispatches the matching
     * detection (by class ID) to each tracker instance.
     *
     * @param detections_message ROS vision_msgs/Detection2DArray message.
     */
    void DetectionsCallback(
        const vision_msgs::msg::Detection2DArray::SharedPtr detections_message);

    // Tracking image publishing
    // -------------------------------------------------------------------------

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

    /** @brief Shared pointer to the object detections overlay image publisher.
     */
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

    void PublishImage();

    // Task status
    // -------------------------------------------------------------------------

    /** @brief Mission tasks status subscription. */
    rclcpp::Subscription<avt_341_msgs::msg::MissionTaskStatus>::SharedPtr
        task_status_subscription_;

    /**
     * @brief Mission task status subscription callback. A non-empty tracked
     * vehicle adds or re-targets a tracker without disturbing the others.
     *
     * @param task_status_message ROS avt_341_msgs/MissionTaskStatus message.
     */
    void TaskStatusCallback(
        avt_341_msgs::msg::MissionTaskStatus::SharedPtr task_status_message);

    // Reset handling
    // -------------------------------------------------------------------------

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reset_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reset_ack_publisher_;
    bool reset_called_ = false;
    void ResetCallback(std_msgs::msg::String::SharedPtr msg);

    // Target contacts (shared topic, injected into every tracker)
    // -------------------------------------------------------------------------

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr target_contacts_publisher_;

    // Tracker information
    // -------------------------------------------------------------------------

    rclcpp::Publisher<avt_341_msgs::msg::TrackerInfo>::SharedPtr
        info_publisher_;

    /** @brief Publishes one TrackerInfo message per tracker on the shared
     *         info topic, identified by header.frame_id = target class. */
    void TrackerInfoCallback();

    /** @brief The timer for the tracker information publishing callback. */
    rclcpp::TimerBase::SharedPtr info_timer_;

    // Timers (remain in the node; each tick iterates the trackers)
    // -------------------------------------------------------------------------

    rclcpp::TimerBase::SharedPtr estimator_timer_;

    void EstimatorTimerCallback();

    rclcpp::TimerBase::SharedPtr tracking_timer_;

    void TrackingTimerCallback();

    double execution_time_ = -1.0;

    // Target selection service
    // -------------------------------------------------------------------------

    /** @brief Service server for the target selection service. */
    rclcpp::Service<avt_341_msgs::srv::SetTarget>::SharedPtr
        set_target_service_server_;

    /**
     * @brief Target selection service callback. Adds or re-targets a tracker
     * for the requested target ID without disturbing the others.
     *
     * @param request Request containing the selected target ID.
     * @param response Response containing the service outcome and report
     * message.
     */
    void SetTargetServiceCallback(
        const std::shared_ptr<avt_341_msgs::srv::SetTarget::Request> request,
        std::shared_ptr<avt_341_msgs::srv::SetTarget::Response> response);

    // Integrated LiDAR obstacle detector
    // -------------------------------------------------------------------------

    /** @brief Runs the obstacle detection pipeline on the latest point cloud
     *         and directly updates latest_obstacle_markers_. Called from
     *         PointCloudCallback so the markers are ready before the next
     *         tracking timer tick. */
    void RunObstacleDetection(
        const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg);

    /** @brief Builds and publishes (and stores) a DELETEALL MarkerArray. */
    void PublishObstacleDeleteAll(const std_msgs::msg::Header& header);

    /** @brief Builds markers from curr_boxes_ and publishes them. */
    void PublishObstacleMarkers(const std_msgs::msg::Header& header);

    /** @brief The obstacle detector algorithm (filter / cluster / track). */
    std::shared_ptr<avt_341::perception::LidarObstacleDetector<pcl::PointXYZ>>
        obstacle_detector_;

    /** @brief Rolling counter used to assign unique IDs to new boxes. */
    size_t obstacle_id_ = 0;

    /** @brief Box list from the previous obstacle detection frame.
     *         Used by the Hungarian-algorithm tracker. */
    std::vector<Box> prev_boxes_;

    /** @brief Box list built during the current obstacle detection frame. */
    std::vector<Box> curr_boxes_;

    /** @brief Publishes the obstacle bounding-box markers. */
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        obstacle_bboxes_publisher_;

    /** @brief Publishes the ground-classified point cloud (optional). */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        obstacle_ground_cloud_publisher_;

    /** @brief Publishes the non-ground obstacle point cloud (optional). */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        obstacle_clusters_cloud_publisher_;

    /** @brief Latest MarkerArray produced by the integrated obstacle detector.
     *         Populated synchronously in PointCloudCallback. */
    visualization_msgs::msg::MarkerArray latest_obstacle_markers_;

    /** @brief True once at least one MarkerArray (including DELETEALL) has
     *         been produced by the obstacle detector. */
    bool has_obstacle_markers_ = false;
};

}  // namespace perception
}  // namespace avt_341
