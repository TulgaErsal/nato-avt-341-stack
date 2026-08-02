#include "avt_341_nav/perception/layers/camera_seg_pc_layer.hpp"

#include "avt_341_nav/core/ros_msg_validation.hpp"
#include "avt_341_nav/core/string_utils.hpp"
#include "avt_341_nav/node/node_utils.h"
#include "avt_341_nav/perception/layers/camera_projection_utils.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "sensor_msgs/msg/point_field.hpp"

#ifdef GTE_ROS_HUMBLE
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#else
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#endif

namespace avt_341_nav::perception
{
    namespace
    {
        struct ProjectedPoint
        {
            float x = 0.0F;
            float y = 0.0F;
            float z = 0.0F;
            float segmentation = 0.0F;
        };

    }

    CameraSegPcLayer::CameraSegPcLayer(
        const rclcpp::Node::SharedPtr& node_ref,
        const std::shared_ptr<node::TfInterface>& tf,
        const PerceptionSettings& settings,
        const std::string& label,
        const std::shared_ptr<core::ComputeTimeRecorder>& compute_time_recorder,
        const avt_341_nav::params::perception::Params::CameraSegPcLayer& params)
            : PointCloudLayer(
                node_ref, tf, settings, label, compute_time_recorder,
                "", "", params.contribute_occupancy,
                params.contribute_segmentation, false),
              projection_tf_(tf),
              use_n_last_scans_(static_cast<std::size_t>(params.use_n_last_scans)),
              pc_move_threshold_(params.pc_move_threshold)
    {
        const std::string lidar_frame_in = settings.clear_method.lidar_frame.empty()
            ? "lidar"
            : settings.clear_method.lidar_frame;
        lidar_frame_ = core::CombineTfParts(
            node::GetLeadingNodeNamespace(node_ref_), lidar_frame_in);
        SetupSubscriptions(params);
    }

    void CameraSegPcLayer::SetupSubscriptions(
        const avt_341_nav::params::perception::Params::CameraSegPcLayer& params)
    {
        point_cloud_topic_ = params.point_cloud_topic;
        segmentation_topic_ = params.segmentation_topic;
        camera_info_topic_ = params.info_topic;

        if (point_cloud_topic_.empty()
            || segmentation_topic_.empty()
            || camera_info_topic_.empty())
        {
            is_enabled_ = false;
            return;
        }
        is_enabled_ = true;
        has_segmentation_ = true;

        camera_info_sub_ =
            node_ref_->create_subscription<sensor_msgs::msg::CameraInfo>(
                camera_info_topic_, 10,
                std::bind(
                    &CameraSegPcLayer::CameraInfoCallback, this,
                    std::placeholders::_1));
        point_cloud_sub_ =
            node_ref_->create_subscription<sensor_msgs::msg::PointCloud2>(
                point_cloud_topic_, 10,
                std::bind(
                    &CameraSegPcLayer::LidarPointCloudCallback, this,
                    std::placeholders::_1));
        segmentation_sub_ =
            node_ref_->create_subscription<sensor_msgs::msg::Image>(
                segmentation_topic_, 10,
                std::bind(
                    &CameraSegPcLayer::SegmentationCallback, this,
                    std::placeholders::_1));
    }

    void CameraSegPcLayer::CameraInfoCallback(
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
    {
        camera_model_.fromCameraInfo(msg);

        try
        {
            camera_projection::ValidateDistortionModel(camera_model_, msg->width, msg->height);
        }
        catch (const std::exception& exception)
        {
            RCLCPP_WARN_THROTTLE(
                node_ref_->get_logger(), *node_ref_->get_clock(),
                THROTTLE_LOG_PERIOD * 1000.0,
                "CameraSegPcLayer: %s Ignoring camera info.",
                exception.what());
            camera_info_received_ = false;
            return;
        }

        cached_camera_info_ = *msg;
        camera_info_received_ = true;
    }

    void CameraSegPcLayer::LidarPointCloudCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg)
    {
        AddLidarScan(cloud_msg);
    }

    bool CameraSegPcLayer::AddLidarScan(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg)
    {

        const rclcpp::Time scan_stamp(cloud_msg->header.stamp);
        if (!retained_scans_.empty() && scan_stamp <= rclcpp::Time(retained_scans_.back()->header.stamp))
        {
            RCLCPP_WARN_THROTTLE(
                node_ref_->get_logger(), *node_ref_->get_clock(),
                THROTTLE_LOG_PERIOD * 1000.0,
                "CameraSegPcLayer: out-of-order lidar scan received, skipping.");
            return false;
        }

        if (use_n_last_scans_ == 1)
        {
            retained_scans_.clear();
            retained_scans_.push_back(cloud_msg);
            return true;
        }

        if (pc_move_threshold_ > 0.0)
        {
            const auto lidar_pose = projection_tf_->lookup_transform("map", lidar_frame_, scan_stamp);
            const double lidar_x = lidar_pose.transform.translation.x;
            const double lidar_y = lidar_pose.transform.translation.y;
            if (has_last_retained_lidar_position_
                && !camera_projection::MeetsPlanarScanDistance(
                    lidar_x, lidar_y,
                    last_retained_lidar_x_, last_retained_lidar_y_,
                    pc_move_threshold_))
            {
                return false;
            }

            last_retained_lidar_x_ = lidar_x;
            last_retained_lidar_y_ = lidar_y;
            has_last_retained_lidar_position_ = true;
        }

        retained_scans_.push_back(cloud_msg);
        while (retained_scans_.size() > use_n_last_scans_)
        {
            retained_scans_.pop_front();
        }
        return true;
    }

    void CameraSegPcLayer::SegmentationCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr& seg_msg)
    {
        ProcessSegmentation(seg_msg);
    }

    void CameraSegPcLayer::ProcessSegmentation(
        const sensor_msgs::msg::Image::ConstSharedPtr& seg_msg)
    {
        if (!camera_info_received_)
        {
            RCLCPP_WARN_THROTTLE(
                node_ref_->get_logger(), *node_ref_->get_clock(),
                THROTTLE_LOG_PERIOD * 1000.0,
                "CameraSegPcLayer: segmentation image received but no CameraInfo yet, skipping.");
            return;
        }

        try
        {
            core::ValidateImageWithCameraInfo(seg_msg, cached_camera_info_, EXPECTED_SEG_FORMAT);
        }
        catch (const std::exception& exception)
        {
            RCLCPP_WARN_THROTTLE(
                node_ref_->get_logger(), *node_ref_->get_clock(),
                THROTTLE_LOG_PERIOD * 1000.0,
                "CameraSegPcLayer: %s Skipping segmentation image.",
                exception.what());
            return;
        }

        const uint32_t width = seg_msg->width;
        const uint32_t height = seg_msg->height;
        const std::string& camera_frame = cached_camera_info_.header.frame_id;
        const rclcpp::Time image_stamp(seg_msg->header.stamp);
        const rclcpp::Time max_scan_stamp =
            image_stamp + rclcpp::Duration::from_seconds(SCAN_STAMP_MARGIN_SEC);
        std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> eligible_scans;
        eligible_scans.reserve(retained_scans_.size());
        for (const auto& scan : retained_scans_)
        {
            if (rclcpp::Time(scan->header.stamp) <= max_scan_stamp)
            {
                eligible_scans.push_back(scan);
            }
        }
        if (eligible_scans.empty())
        {
            RCLCPP_WARN_THROTTLE(
                node_ref_->get_logger(), *node_ref_->get_clock(),
                THROTTLE_LOG_PERIOD * 1000.0,
                "CameraSegPcLayer: no retained lidar scans are old enough for the segmentation image.");
            return;
        }

        const std::size_t pixel_count = static_cast<std::size_t>(width) * height;
        std::vector<float> nearest_depth(
            pixel_count, std::numeric_limits<float>::infinity());
        std::vector<ProjectedPoint> projected_points(pixel_count);
        std::size_t projected_count = 0;

        for (const auto& scan : eligible_scans)
        {
            const auto scan_to_camera = projection_tf_->lookup_transform(
                camera_frame, image_stamp,
                scan->header.frame_id, rclcpp::Time(scan->header.stamp),
                "map");
            if (scan_to_camera.header.frame_id.empty())
            {
                continue;
            }

            sensor_msgs::msg::PointCloud2 camera_cloud;
            try
            {
                tf2::doTransform(*scan, camera_cloud, scan_to_camera);

                sensor_msgs::PointCloud2ConstIterator<float> iter_x(camera_cloud, "x");
                sensor_msgs::PointCloud2ConstIterator<float> iter_y(camera_cloud, "y");
                sensor_msgs::PointCloud2ConstIterator<float> iter_z(camera_cloud, "z");
                for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
                {
                    const float x = *iter_x;
                    const float y = *iter_y;
                    const float z = *iter_z;
                    if (!std::isfinite(x) || !std::isfinite(y)
                        || !std::isfinite(z) || z <= 0.0F)
                    {
                        continue;
                    }

                    const auto pixel = camera_projection::ProjectToRawImage(
                        camera_model_, cv::Point3d(x, y, z), width, height);
                    if (!pixel.has_value())
                    {
                        continue;
                    }

                    const auto u = static_cast<uint32_t>(pixel->x);
                    const auto v = static_cast<uint32_t>(pixel->y);
                    const std::size_t pixel_index =
                        static_cast<std::size_t>(v) * width + u;
                    if (z >= nearest_depth[pixel_index])
                    {
                        continue;
                    }

                    if (!std::isfinite(nearest_depth[pixel_index]))
                    {
                        ++projected_count;
                    }
                    nearest_depth[pixel_index] = z;
                    projected_points[pixel_index] = {
                        x, y, z,
                        static_cast<float>(
                            seg_msg->data[static_cast<std::size_t>(v) * seg_msg->step + u])};
                }
            }
            catch (const std::exception& exception)
            {
                RCLCPP_WARN_THROTTLE(
                    node_ref_->get_logger(), *node_ref_->get_clock(),
                    THROTTLE_LOG_PERIOD * 1000.0,
                    "CameraSegPcLayer: failed to transform or project lidar scan: %s",
                    exception.what());
            }
        }

        if (projected_count == 0)
        {
            return;
        }

        auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud->header = seg_msg->header;
        cloud->header.frame_id = camera_frame;
        cloud->height = 1;
        cloud->width = static_cast<uint32_t>(projected_count);
        cloud->is_bigendian = false;
        cloud->is_dense = true;

        sensor_msgs::PointCloud2Modifier modifier(*cloud);
        modifier.setPointCloud2Fields(
            4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            pc_seg_channel_.c_str(), 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(projected_count);

        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_seg(*cloud, pc_seg_channel_);
        for (std::size_t pixel_index = 0; pixel_index < pixel_count; ++pixel_index)
        {
            if (!std::isfinite(nearest_depth[pixel_index]))
            {
                continue;
            }
            const ProjectedPoint& point = projected_points[pixel_index];
            *iter_x = point.x;
            *iter_y = point.y;
            *iter_z = point.z;
            *iter_seg = point.segmentation;
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_seg;
        }

        PointCloudCallback(cloud);
    }

    void CameraSegPcLayer::Reset()
    {
        PointCloudLayer::Reset();
        retained_scans_.clear();
        has_last_retained_lidar_position_ = false;
        last_retained_lidar_x_ = 0.0;
        last_retained_lidar_y_ = 0.0;
    }

    std::string CameraSegPcLayer::ToString() const
    {
        return "[CameraSegPcLayer] id: " + label_
            + ", point_cloud_topic: " + point_cloud_topic_
            + ", info_topic: " + camera_info_topic_
            + ", seg_topic: " + segmentation_topic_
            + ", retained_scans: " + std::to_string(retained_scans_.size())
            + "/" + std::to_string(use_n_last_scans_)
            + ", pc_move_threshold: "
            + std::to_string(pc_move_threshold_) + "m";
    }
}
