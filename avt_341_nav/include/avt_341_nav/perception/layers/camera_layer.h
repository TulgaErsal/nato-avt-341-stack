#ifndef AVT_341_CAMERA_LAYER_H
#define AVT_341_CAMERA_LAYER_H

#include "avt_341/perception/layers/point_cloud_layer.h"

#ifdef GTE_ROS_JAZZY
#include <image_geometry/pinhole_camera_model.hpp>
#else
#include <image_geometry/pinhole_camera_model.h>
#endif


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/types.hpp>
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <rclcpp/rclcpp.hpp>

namespace avt_341::perception
{
    class CameraLayer : public PointCloudLayer
    {
    public:
        CameraLayer(
            const rclcpp::Node::SharedPtr& node_ref,
            const std::shared_ptr<node::TfInterface>& tf,
            const PerceptionSettings& settings,
            const std::string & label,
            const std::shared_ptr<core::ComputeTimeRecorder>& compute_time_recorder,
            const avt_341::params::perception::Params::CameraLayer& params
            );

        std::string ToString() const override;

    protected:
        static constexpr double THROTTLE_LOG_PERIOD = 2.0;
        static constexpr std::string_view EXPECTED_DEPTH_FORMAT = "32FC1";
        static constexpr std::string_view EXPECTED_SEG_FORMAT = "mono8";

    private:
        using ImageSyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

        void SetupCameraSubscriptions(
            const avt_341::params::perception::Params::CameraLayer& params);

        void CameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);

        void SyncedImageCallback(
            const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
            const sensor_msgs::msg::Image::ConstSharedPtr& seg_msg);

        void DepthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);

        void ProcessToPointCloud(
            const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
            const sensor_msgs::msg::Image::ConstSharedPtr& seg_msg = nullptr);

        void RebuildRayCache();

        image_geometry::PinholeCameraModel camera_model_;
        bool camera_info_received_ = false;

        std::vector<cv::Point3d> ray_cache_;
        sensor_msgs::msg::CameraInfo cached_camera_info_;

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> seg_sub_;
        std::shared_ptr<message_filters::Synchronizer<ImageSyncPolicy>> sync_;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_only_sub_;

        std::string depth_img_topic_;
        std::string seg_img_topic_;
        std::string camera_info_topic_;
    };
}

#endif

