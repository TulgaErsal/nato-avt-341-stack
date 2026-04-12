#include "avt_341/perception/layers/camera_layer.h"
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace avt_341::perception
{
    CameraLayer::CameraLayer(
        const std::shared_ptr<node::NodeProxy>& node_ref,
        const CostmapSettings& cm_settings,
        const std::string& label)
            : PointCloudLayer(node_ref, cm_settings, label)
    {
        camera_model_ = std::make_unique<image_geometry::PinholeCameraModel>();
        SetupCameraSubscriptions();
    }

    void CameraLayer::SetupCameraSubscriptions()
    {
        std::string depth_img_topic, seg_img_topic, camera_info_topic;
        node_ref_->get_parameter("~" + label_ + "_depth_image_topic", depth_img_topic, std::string(""));
        node_ref_->get_parameter("~" + label_ + "_seg_image_topic", seg_img_topic, std::string(""));
        node_ref_->get_parameter("~" + label_ + "_camera_info_topic", camera_info_topic, std::string(""));

        if (depth_img_topic.empty() || camera_info_topic.empty())
        {
            is_valid_ = false;
            return;
        }

        std::shared_ptr<rclcpp::Node> raw_node = node_ref_->get_raw_node();

        camera_info_sub_ = raw_node->create_subscription<msg::CameraInfo>(
            camera_info_topic, 10,
            std::bind(&CameraLayer::CameraInfoCallback, this, std::placeholders::_1));

        has_segmentation_ = !seg_img_topic.empty();
        if (has_segmentation_)
        {
            // Use approximate time synchronizer for depth + segmentation images
            depth_sub_ = std::make_shared<message_filters::Subscriber<msg::Image>>(
                raw_node, depth_img_topic);
            seg_sub_ = std::make_shared<message_filters::Subscriber<msg::Image>>(
                raw_node, seg_img_topic);

            sync_ = std::make_shared<message_filters::Synchronizer<ImageSyncPolicy>>(
                ImageSyncPolicy(10), *depth_sub_, *seg_sub_);
            sync_->registerCallback(
                std::bind(&CameraLayer::SyncedImageCallback, this,
                          std::placeholders::_1, std::placeholders::_2));
        }
        else
        {
            // No segmentation topic: subscribe to depth only
            depth_only_sub_ = raw_node->create_subscription<msg::Image>(
                depth_img_topic, 10,
                std::bind(&CameraLayer::DepthImageCallback, this, std::placeholders::_1));
        }

        node_ref_->log_info("CameraLayer subscriptions: depth=%s, seg=%s, info=%s",
            depth_img_topic.c_str(),
            seg_img_topic.empty() ? "(none)" : seg_img_topic.c_str(),
            camera_info_topic.c_str());
    }

    void CameraLayer::CameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
    {
        // fromCameraInfo returns true when calibration parameters changed
        bool params_changed = camera_model_->fromCameraInfo(msg);

        if (!camera_info_received_ || params_changed)
        {
            RebuildRayCache();
            cached_camera_info_ = *msg;
            camera_info_received_ = true;
            node_ref_->log_info("CameraLayer: camera model updated (%ux%u)", msg->width, msg->height);
        }
    }

    void CameraLayer::RebuildRayCache()
    {
        const auto width = camera_model_->cameraInfo().width;
        const auto height = camera_model_->cameraInfo().height;
        ray_cache_.resize(width * height);

        for (uint32_t v = 0; v < height; ++v)
        {
            for (uint32_t u = 0; u < width; ++u)
            {
                // projectPixelTo3dRay accounts for distortion via the rectified projection.
                // The returned ray has z = 1.0.
                ray_cache_[v * width + u] = camera_model_->projectPixelTo3dRay(
                    cv::Point2d(static_cast<double>(u), static_cast<double>(v)));
            }
        }
    }

    void CameraLayer::SyncedImageCallback(
        const msg::Image::ConstSharedPtr& depth_msg,
        const msg::Image::ConstSharedPtr& seg_msg)
    {
        ProcessDepthImage(depth_msg, seg_msg);
    }

    void CameraLayer::DepthImageCallback(const msg::Image::ConstSharedPtr& depth_msg)
    {
        ProcessDepthImage(depth_msg, nullptr);
    }

    void CameraLayer::ProcessDepthImage(
        const msg::Image::ConstSharedPtr& depth_msg,
        const msg::Image::ConstSharedPtr& seg_msg)
    {
        // Guard: must have received a CameraInfo before we can project pixels
        if (!camera_info_received_)
        {
            node_ref_->log_warning_throttle(2.0, "CameraLayer: depth image received but no CameraInfo yet, skipping.");
            return;
        }

        const uint32_t width = depth_msg->width;
        const uint32_t height = depth_msg->height;

        // Verify the ray cache matches the image dimensions
        if (ray_cache_.size() != static_cast<size_t>(width * height))
        {
            node_ref_->log_warning_throttle(2.0,
                "CameraLayer: ray cache size (%zu) does not match image (%ux%u), skipping.",
                ray_cache_.size(), width, height);
            return;
        }

        // Depth must be 32FC1
        if (depth_msg->encoding != "32FC1")
        {
            node_ref_->log_warning_throttle(2.0,
                "CameraLayer: unsupported depth encoding '%s', expected 32FC1.",
                depth_msg->encoding.c_str());
            return;
        }

        // Determine segmentation encoding if present
        const bool has_seg = (seg_msg != nullptr);
        if (has_seg)
        {
            // Segmentation must be 8UC1
            if (seg_msg->encoding != "8UC1")
            {
                node_ref_->log_warning_throttle(2.0,
                    "CameraLayer: unsupported segmentation encoding '%s', expected 8UC1.",
                    seg_msg->encoding.c_str());
                return;
            }

            if (seg_msg->width != width || seg_msg->height != height)
            {
                node_ref_->log_warning_throttle(2.0,
                    "CameraLayer: segmentation image size (%ux%u) doesn't match depth (%ux%u).",
                    seg_msg->width, seg_msg->height, width, height);
                return;
            }
        }

        // Collect valid depth pixels (depth value + pixel coordinates)
        struct ValidPixel { float depth; uint32_t v; uint32_t u; };
        std::vector<ValidPixel> valid_pixels;
        valid_pixels.reserve(width * height);

        for (uint32_t v = 0; v < height; ++v)
        {
            for (uint32_t u = 0; u < width; ++u)
            {
                float depth;
                std::memcpy(&depth, &depth_msg->data[v * depth_msg->step + u * sizeof(float)], sizeof(float));
                if (std::isfinite(depth) && depth > 0.0f)
                {
                    valid_pixels.push_back({depth, v, u});
                }
            }
        }

        if (valid_pixels.empty())
            return;

        const size_t valid_count = valid_pixels.size();

        // Build the PointCloud2 message
        auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud->header = depth_msg->header;
        cloud->height = 1;
        cloud->width = static_cast<uint32_t>(valid_count);
        cloud->is_bigendian = false;
        cloud->is_dense = true;

        // Set up fields: x, y, z, and optionally segmentation
        sensor_msgs::PointCloud2Modifier modifier(*cloud);
        if (has_seg)
        {
            modifier.setPointCloud2Fields(4,
                "x", 1, msg::PointField::FLOAT32,
                "y", 1, msg::PointField::FLOAT32,
                "z", 1, msg::PointField::FLOAT32,
                pc_seg_channel_.c_str(), 1, sensor_msgs::msg::PointField::FLOAT32);
        }
        else
        {
            modifier.setPointCloud2FieldsByString(1, "xyz");
        }
        modifier.resize(valid_count);

        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");

        // Optional segmentation iterator
        std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> iter_seg;
        if (has_seg)
        {
            iter_seg = std::make_unique<sensor_msgs::PointCloud2Iterator<float>>(*cloud, pc_seg_channel_);
        }

        // Project each valid pixel to 3D using the cached rays
        for (const auto& px : valid_pixels)
        {
            const cv::Point3d& ray = ray_cache_[px.v * width + px.u];
            // ray.z == 1.0, so depth directly scales the ray
            *iter_x = static_cast<float>(ray.x * px.depth);
            *iter_y = static_cast<float>(ray.y * px.depth);
            *iter_z = static_cast<float>(ray.z * px.depth);

            if (has_seg && iter_seg)
            {
                **iter_seg = static_cast<float>(seg_msg->data[px.v * seg_msg->step + px.u]);
                ++(*iter_seg);
            }

            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        // Forward the generated point cloud to the base PointCloudLayer
        PointCloudCallback(cloud);
    }
}
