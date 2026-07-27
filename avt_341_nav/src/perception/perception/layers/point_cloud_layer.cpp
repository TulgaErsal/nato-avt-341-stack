#include "avt_341/perception/layers/point_cloud_layer.h"
#include "avt_341/perception/clearing_methods/clearing_methods_factory.h"
#include <chrono>
#include <thread>

#include "sensor_msgs/point_cloud_conversion.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace avt_341::perception
{
PointCloudLayer::PointCloudLayer(
    const rclcpp::Node::SharedPtr& node_ref,
    const std::shared_ptr<node::TfInterface>& tf,
    const PerceptionSettings& settings,
    const std::string & label,
    const std::shared_ptr<core::ComputeTimeRecorder>& compute_time_recorder,
    const std::string& point_cloud_topic,
    const std::string& clear_only_points_topic,
    const bool contribute_occupancy,
    const bool contribute_segmentation,
    const bool setup_point_cloud_subscriptions
    )
        : CostmapLayer(
            node_ref, settings, label, compute_time_recorder,
            contribute_occupancy, contribute_segmentation),
          tf_(tf)
{
	const double pc_callback_warn_dur =
        settings.point_cloud_layer.callback_warn_time;
	pc_seg_channel_ = settings.point_cloud_layer.segmentation_channel;

	pc_section_id_ = label + "/pc_callback";
	core::RunningStatsConfig section_config;
	section_config.window_num_samples = 40;
	section_config.threshold_check = pc_callback_warn_dur;
	compute_time_recorder_->Configure(pc_section_id_, section_config);

    SetupGridClearingMethod(settings.clear_method);
    SetupPointCloudFilter(
        ParseFilterConfig(settings.point_cloud_layer.filter),
        ParseFilterConfig(settings.clear_method.filter));
    if (setup_point_cloud_subscriptions) {
        SetupPcSubscriptions(point_cloud_topic, clear_only_points_topic);
    }
}

void PointCloudLayer::SetupPcSubscriptions(
    const std::string& point_cloud_topic,
    const std::string& clear_only_points_topic)
{
    pc_topic_id_ = point_cloud_topic;

    if (pc_topic_id_.empty())
    {
        is_enabled_ = false;
        return;
    }

    pc_sub_ = node_ref_->create_subscription<sensor_msgs::msg::PointCloud2>(
        pc_topic_id_,
        10,
        std::bind(&PointCloudLayer::PointCloudCallback, this, std::placeholders::_1)
    );

    if (!clear_only_points_topic.empty())
    {
        pc_ground_sub_ = node_ref_->create_subscription<sensor_msgs::msg::PointCloud2>(
            clear_only_points_topic,
            10,
            std::bind(&PointCloudLayer::ClearOnlyPointsCallback, this, std::placeholders::_1));
    }
}

PointCloudFilterConfig PointCloudLayer::ParseFilterConfig(
    const GeneratedPerceptionParams::PointCloudLayer::Filter& params) {
	PointCloudFilterConfig config;
    config.enable_dist_filter = params.enable_dist_filter;
    config.max_dist = params.max_dist;
    config.min_dist = params.min_dist;
    config.min_hfov = params.min_hfov;
    config.max_hfov = params.max_hfov;
    config.max_height_clearance = params.max_height_clearance;
	return config;
}

PointCloudFilterConfig PointCloudLayer::ParseFilterConfig(
    const GeneratedPerceptionParams::ClearMethod::Filter& params) {
	PointCloudFilterConfig config;
    config.enable_dist_filter = params.enable_dist_filter;
    config.max_dist = params.max_dist;
    config.min_dist = params.min_dist;
    config.min_hfov = params.min_hfov;
    config.max_hfov = params.max_hfov;
    config.max_height_clearance = params.max_height_clearance;
	return config;
}

std::shared_ptr<sensor_msgs::msg::PointCloud> PointCloudLayer::RegisterPc2Msg(const sensor_msgs::msg::PointCloud2::SharedPtr & rcv_cloud) {

    std::shared_ptr<sensor_msgs::msg::PointCloud2> pc2_ptr = rcv_cloud;

    if (rcv_cloud->header.frame_id != "odom" && rcv_cloud->header.frame_id != "map") {
        pc2_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>();
        if (!tf_->transform_cloud(*rcv_cloud, *pc2_ptr, "map")) {
            return nullptr;
        }
    }

    std::shared_ptr<sensor_msgs::msg::PointCloud> pc_ptr = std::make_shared<sensor_msgs::msg::PointCloud>();
    if (!sensor_msgs::convertPointCloud2ToPointCloud(*pc2_ptr, *pc_ptr)) {
        return nullptr;
    }

    return pc_ptr;
}

void PointCloudLayer::PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr rcv_cloud) {

    auto recording = compute_time_recorder_->RecordScope(pc_section_id_);

    std::shared_ptr<sensor_msgs::msg::PointCloud> pc = RegisterPc2Msg(rcv_cloud);
    const std::string veh_frame = current_odom_.child_frame_id;

    if (veh_frame.empty() || pc == nullptr) {
        recording.Cancel();
        return;
    }

    if (clr_only_pc_ != nullptr) {
        geometry_msgs::msg::PoseStamped origin_pose = tf_->lookup_pose("map", veh_frame, clr_only_pc_->header.stamp);
        ClearPoints(clr_only_pc_, origin_pose.pose);
        clr_only_pc_ = nullptr;
    }

    geometry_msgs::msg::PoseStamped origin_pose = tf_->lookup_pose("map", veh_frame, rcv_cloud->header.stamp);
    ProcessPoints(pc, origin_pose.pose);
}

void PointCloudLayer::ClearOnlyPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr rcv_cloud) {
    clr_only_pc_ = RegisterPc2Msg(rcv_cloud);
}


void PointCloudLayer::SetupPointCloudFilter(
    const PointCloudFilterConfig& point_cloud_config,
    const PointCloudFilterConfig& clearing_config) {
    pc_filter.SetConfig(point_cloud_config);
    pc_cm_filter.SetConfig(clearing_config);

    RCLCPP_INFO(node_ref_->get_logger(), "Point cloud culling: %s", pc_filter.GetDescription().c_str());

    RCLCPP_INFO(node_ref_->get_logger(), "Point cloud culling (extra for grid clearing): %s", pc_cm_filter.GetDescription().c_str());
}

void PointCloudLayer::ProcessPoints(const std::shared_ptr<sensor_msgs::msg::PointCloud>& pc_ptr, const geometry_msgs::msg::Pose& vehicle_pose, const bool clear_only) {

    if (is_resetting_) {
        return;
    }

    // Filtered point cloud for normal occupancy addition
    auto filtered_pc = pc_filter.Filter(pc_ptr, vehicle_pose);

    // Additional filtering for clearing methods if desired
    auto filtered_cms_pc = clear_methods_.empty() ? filtered_pc : pc_cm_filter.Filter(filtered_pc, vehicle_pose);

    for (auto& cm : clear_methods_) {
        cm->ClearOccupancy(*filtered_cms_pc);
    }

    if (clear_only) {
        return;
    }

    AddOccupancy(
        *filtered_pc, cells_, settings_.costmap.dilation.enabled);
    for (auto& cm : clear_methods_) {
        cm->OnOccupancyAdded(*filtered_cms_pc, vehicle_pose.position);
    }

}

void PointCloudLayer::AddOccupancy(const sensor_msgs::msg::PointCloud& point_cloud, std::vector< std::vector<Cell> >& cells, bool dilate) {

    bool has_segmentation_local = !point_cloud.channels.empty() && point_cloud.channels[0].name == pc_seg_channel_;
    has_segmentation_ = has_segmentation_local || has_segmentation_;

    const auto llx = settings_.size_info().llx;
    const auto lly = settings_.size_info().lly;
    const auto res = settings_.size_info().res;
    const auto nx = settings_.nx();
    const auto ny = settings_.ny();

    // fill the cells with highest and lowest points
    for (int i = 0; i < point_cloud.points.size(); i++) {
        if (!(point_cloud.points[i].x == 0.0 && point_cloud.points[i].y == 0.0)) {
            int xi = (int)floor((point_cloud.points[i].x - llx) / res);
            int yi = (int)floor((point_cloud.points[i].y - lly) / res);
            if (xi >= 0 && xi < nx && yi >= 0 && yi < ny) {
                Cell& cell = cells[yi][xi];
                grid_update_region_.UpdateBounds(xi,yi);
                const float original_slope = Slope(cell);
                float h = point_cloud.points[i].z;

                if (h > cell.high.val) {
                    cell.high.val = h;
                    cell.high.age = 0.0f;
                }
                if (h < cell.low.val) {
                    cell.low.val = h;
                    cell.low.age = 0.0f;
                }
                if (has_segmentation_local) {
                    cell.terrain_seg = static_cast<int>(point_cloud.channels[0].values[i]);
                }

                // CTG 5/8/25, add calculations necessary for tracking RMS
                cell.summed_elev += h;
                cell.num_points += 1;
                if (cell.num_points > 0) {
                    cell.avg_elev = cell.summed_elev / cell.num_points;
                    float dh = h - cell.avg_elev;
                    cell.sum_of_squares += dh * dh;
                    cell.rms = sqrtf(cell.sum_of_squares / cell.num_points);
                }
                else {
                    cell.avg_elev = 0.0f;
                    cell.rms = 0.0f;
                }

                if (dilate) {
                    DilateCell(cells, xi, yi, original_slope);
                }
            }
        }
    }
}

void PointCloudLayer::ClearPoints(const std::shared_ptr<sensor_msgs::msg::PointCloud>& pc_ptr, const geometry_msgs::msg::Pose& vehicle_pose) {
    ProcessPoints(pc_ptr, vehicle_pose, true);
}

void PointCloudLayer::Reset() {
    is_resetting_ = true;

    while (HasData()) {
    	Clear();
    	for (const auto& cm : clear_methods_) {
    		cm->Reset();
    	}
    	std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    is_resetting_ = false;
}

void PointCloudLayer::SetupGridClearingMethod(
    const ClearMethodSettings& params) {
    clear_methods_ = ClearingMethodFactory::CreateClearingMethods(
        node_ref_, tf_, cells_, params, settings_, this);
}

void PointCloudLayer::Visualize()
{
    for (const auto& cm : clear_methods_) {
        cm->Visualize();
    }
}

std::string PointCloudLayer::ToString() const
{
    return "[PointCloudLayer] id: " + label_
        + ", pc_topic: " + pc_topic_id_;
}
}
