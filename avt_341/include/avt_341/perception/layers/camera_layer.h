#ifndef AVT_341_CAMERA_LAYER_H
#define AVT_341_CAMERA_LAYER_H

#include "avt_341/perception/layers/point_cloud_layer.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/types.hpp>

namespace image_geometry
{
    class PinholeCameraModel;
}

namespace avt_341::perception
{
    class CameraLayer : public PointCloudLayer
    {
    public:
        CameraLayer(
            const std::shared_ptr<node::NodeProxy>& node_ref,
            const CostmapSettings& cm_settings,
            const std::string & label
            );

    private:
        using ImageSyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

        void SetupCameraSubscriptions();

        void CameraInfoCallback(const msg::CameraInfo::ConstSharedPtr& msg);

        void SyncedImageCallback(
            const msg::Image::ConstSharedPtr& depth_msg,
            const msg::Image::ConstSharedPtr& seg_msg);

        void DepthImageCallback(const msg::Image::ConstSharedPtr& depth_msg);

        void ProcessDepthImage(
            const msg::Image::ConstSharedPtr& depth_msg,
            const msg::Image::ConstSharedPtr& seg_msg = nullptr);

        void RebuildRayCache();

        std::unique_ptr<image_geometry::PinholeCameraModel> camera_model_;
        bool camera_info_received_ = false;

        std::vector<cv::Point3d> ray_cache_;
        sensor_msgs::msg::CameraInfo cached_camera_info_;

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> seg_sub_;
        std::shared_ptr<message_filters::Synchronizer<ImageSyncPolicy>> sync_;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_only_sub_;
    };
}

#endif

