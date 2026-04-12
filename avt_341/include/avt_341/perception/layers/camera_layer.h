#ifndef AVT_341_CAMERA_LAYER_H
#define AVT_341_CAMERA_LAYER_H

#include "avt_341/perception/layers/point_cloud_layer.h"

#include <sensor_msgs/msg/camera_info.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/core.hpp>

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

        /// Callback for CameraInfo messages. Updates the pinhole camera model
        /// and rebuilds the cached 3D ray directions when intrinsics change.
        void CameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);

        /// Approximate-time-synced callback for depth and segmentation images.
        /// Projects depth pixels to 3D points using the cached rays and forwards
        /// the resulting PointCloud2 to the base PointCloudLayer.
        void SyncedImageCallback(
            const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
            const sensor_msgs::msg::Image::ConstSharedPtr& seg_msg);

        /// Processes a depth image (without segmentation) and forwards the
        /// resulting PointCloud2 to the base PointCloudLayer.
        void DepthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);

        /// Builds a PointCloud2 from a depth image. If a segmentation image is
        /// provided its per-pixel labels are included as an extra field.
        void ProcessDepthImage(
            const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
            const sensor_msgs::msg::Image::ConstSharedPtr& seg_msg = nullptr);

        /// Rebuilds the per-pixel 3D ray cache from the current camera model.
        void RebuildRayCache();

        image_geometry::PinholeCameraModel camera_model_;
        bool camera_info_received_ = false;

        /// Cached unit-ray per pixel (row-major, size = width * height).
        std::vector<cv::Point3d> ray_cache_;
        /// CameraInfo that was used to build the current ray cache, used to
        /// detect when the intrinsics change and a rebuild is needed.
        sensor_msgs::msg::CameraInfo cached_camera_info_;

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

        /// message_filters subscribers for approximate time sync (depth + seg).
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> seg_sub_;
        std::shared_ptr<message_filters::Synchronizer<ImageSyncPolicy>> sync_;

        /// Plain depth subscription used when no segmentation topic is configured.
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_only_sub_;
    };
}

#endif

