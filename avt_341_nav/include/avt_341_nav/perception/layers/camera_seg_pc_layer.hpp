#ifndef AVT_341_CAMERA_SEG_PC_LAYER_HPP
#define AVT_341_CAMERA_SEG_PC_LAYER_HPP

#include "avt_341_nav/perception/layers/point_cloud_layer.h"

#ifdef GTE_ROS_JAZZY
#include <image_geometry/pinhole_camera_model.hpp>
#else
#include <image_geometry/pinhole_camera_model.h>
#endif

#include <deque>
#include <memory>
#include <string>
#include <string_view>

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace avt_341_nav::perception
{
    class CameraSegPcLayer : public PointCloudLayer
    {
    public:
        /// Create a lidar-to-camera segmentation costmap layer.
        CameraSegPcLayer(
            const rclcpp::Node::SharedPtr& node_ref,
            const std::shared_ptr<node::TfInterface>& tf,
            const PerceptionSettings& settings,
            const std::string& label,
            const std::shared_ptr<core::ComputeTimeRecorder>& compute_time_recorder,
            const avt_341_nav::params::perception::Params::CameraSegPcLayer& params);

        /// Reset the costmap and retained lidar scans.
        void Reset() override;

        /// Return a concise layer configuration summary.
        std::string ToString() const override;

    private:
        static constexpr double THROTTLE_LOG_PERIOD = 2.0;
        static constexpr std::string_view EXPECTED_SEG_FORMAT = "mono8";
        /// Scans up to this much newer than the segmentation image may still be projected.
        static constexpr double SCAN_STAMP_MARGIN_SEC = 0.1;

        /// Subscribe when all required input topics are configured.
        void SetupSubscriptions(
            const avt_341_nav::params::perception::Params::CameraSegPcLayer& params);

        /// Update the segmentation camera model.
        void CameraInfoCallback(
            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);

        /// Consider an incoming lidar scan for retention.
        void LidarPointCloudCallback(
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg);

        /// Project retained lidar scans using the latest segmentation image.
        void SegmentationCallback(
            const sensor_msgs::msg::Image::ConstSharedPtr& seg_msg);

        /// Retain a chronological scan when spatial sampling permits it.
        bool AddLidarScan(
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg);

        /// Build and forward a segmented lidar point cloud.
        void ProcessLidarToPointCloud(
            const sensor_msgs::msg::Image::ConstSharedPtr& seg_msg);

        image_geometry::PinholeCameraModel camera_model_;
        bool camera_info_received_ = false;
        sensor_msgs::msg::CameraInfo cached_camera_info_;

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr segmentation_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;

        std::shared_ptr<node::TfInterface> projection_tf_;
        std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> retained_scans_;

        std::size_t use_n_last_scans_ = 1;
        double pc_move_threshold_ = 0.25;
        bool has_last_retained_lidar_position_ = false;
        double last_retained_lidar_x_ = 0.0;
        double last_retained_lidar_y_ = 0.0;

        std::string point_cloud_topic_;
        std::string segmentation_topic_;
        std::string camera_info_topic_;
        std::string lidar_frame_;
    };
}

#endif // AVT_341_CAMERA_SEG_PC_LAYER_HPP
