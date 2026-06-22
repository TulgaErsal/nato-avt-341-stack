#include "avt_341/perception/layers/point_cloud_layer.h"
#include "avt_341/perception/clearing_methods/clearing_methods_factory.h"
#include <chrono>
#include <thread>

#ifdef ROS_1
#include "sensor_msgs/point_cloud_conversion.h"
#else
#include "sensor_msgs/point_cloud_conversion.hpp"
#endif

namespace avt_341::perception
{
PointCloudLayer::PointCloudLayer(
    const std::shared_ptr<node::NodeProxy>& node_ref,
    const CostmapSettings& cm_settings,
    const std::string & label
    )
        : CostmapLayer(node_ref, cm_settings, label), pc_callback_time_(40)
{
	node_ref_->get_parameter("~pc_callback_warn_time", pc_callback_warn_dur_, 0.1);
	node_ref_->get_parameter("~pc_seg_channel", pc_seg_channel_, std::string("segmentation"));

    SetupGridClearingMethod();
    SetupPointCloudFilter();
    SetupPcSubscriptions();

    node_ref_->params()->add_parameter_callback(std::vector<std::string>{"slope_threshold", "slope_threshold_max"},
    [&](const node::RosParameterEvent & p) {
        thresholds_.Update(
            p.get_value<float>("slope_threshold"),
            p.get_value<float>("slope_threshold_max")
            );
        RecomputeGridDilation();
    });

}

void PointCloudLayer::SetupPcSubscriptions()
{
    std::string clear_only_points_topic;
    node_ref_->get_parameter("~" + label_ + "_topic", pc_topic_id_, std::string(""));
    node_ref_->get_parameter("~" + label_ + "_clear_topic", clear_only_points_topic, std::string(""));

    if (pc_topic_id_.empty())
    {
        is_enabled_ = false;
        return;
    }

    pc_sub_ = node_ref_->create_subscription<msg::PointCloud2>(
        pc_topic_id_,
        10,
        std::bind(&PointCloudLayer::PointCloudCallback, this, std::placeholders::_1)
    );

    if (!clear_only_points_topic.empty())
    {
        pc_ground_sub_ = node_ref_->create_subscription<msg::PointCloud2>(
            clear_only_points_topic,
            10,
            std::bind(&PointCloudLayer::ClearOnlyPointsCallback, this, std::placeholders::_1));
    }
}

ClearMethodRosParameters PointCloudLayer::ParseClearMethodsConfig() const {

    // TODO: Remove after parameter refactor
	ClearMethodRosParameters params;

	// General settings
	node_ref_->get_parameter("~clear_method_type", params.clear_methods_str, std::string("none"));
	node_ref_->get_parameter("~clear_method_visualize", params.visualize, false);
	node_ref_->get_parameter("~clear_method_visualize_range", params.visualization_range, 40.0f);

	// Raytrace clearing
	node_ref_->get_parameter("~clear_method_raytrace_range", params.raytrace_range, 50.0f);
	node_ref_->get_parameter("~clear_method_use_voxels", params.use_voxels, true);
	node_ref_->get_parameter("~clear_method_voxel_height_min", params.voxel_height_min, 0.0f);
	node_ref_->get_parameter("~clear_method_voxel_height_res", params.voxel_height_res, 0.5f);
	node_ref_->get_parameter("~clear_method_immediate_clear_dilation", params.immediate_clr_dilation, true);
	node_ref_->get_parameter("~clear_method_clr_on_scan_below_only", params.clr_on_scan_below_only, false);
	node_ref_->get_parameter("~clear_method_lidar_frame", params.lidar_frame, std::string("lidar"));

	// Raytrace clearing + object filter
	node_ref_->get_parameter("~clear_method_obs_filter_range", params.obj_range_filter, 1.0f);

	// Time and timed no-obs clearing
	node_ref_->get_parameter("~clear_method_sampled_threshold", params.sampled_threshold, 5);
	node_ref_->get_parameter("~clear_method_max_point_age", params.max_point_age, 5.0f);
	node_ref_->get_parameter("~clear_method_no_obs_dist_threshold", params.no_obs_dist_threshold, 0.25f);

	// Channel-based clearing
	node_ref_->get_parameter("~clear_method_channel_to_clear", params.channel_to_clear, std::string("gnd_seg"));
	node_ref_->get_parameter("~clear_method_channel_threshold", params.channel_threshold, 0.5f);

	return params;
}

PointCloudFilterConfig PointCloudLayer::ParseFilterConfig(const std::string &param_prefix) const {

    // TODO: Remove after parameter refactor
	PointCloudFilterConfig config;
	node_ref_->get_parameter("~" + param_prefix + "cull_lidar", config.enable_dist_filter, false);
	node_ref_->get_parameter("~" + param_prefix + "cull_lidar_dist", config.max_dist, -1.0);
	node_ref_->get_parameter("~" + param_prefix + "cull_lidar_dist_min", config.min_dist, -1.0);
	node_ref_->get_parameter("~" + param_prefix + "cull_lidar_hfov_min", config.min_hfov, -180.0);
	node_ref_->get_parameter("~" + param_prefix + "cull_lidar_hfov_max", config.max_hfov, 180.0);
	node_ref_->get_parameter("~" + param_prefix + "overhead_clearance", config.max_height_clearance, -1.0);
	return config;
}

std::shared_ptr<msg::PointCloud> PointCloudLayer::RegisterPc2Msg(const msg::PointCloud2Ptr & rcv_cloud) {

#ifdef ROS_1
    std::shared_ptr<msg::PointCloud2> pc2_ptr = std::make_shared<avt_341::msg::PointCloud2>(*rcv_cloud);
#else
    std::shared_ptr<msg::PointCloud2> pc2_ptr = rcv_cloud;
#endif

    if (rcv_cloud->header.frame_id != "odom" && rcv_cloud->header.frame_id != "map") {
        pc2_ptr = std::make_shared<msg::PointCloud2>();
        if (!node_ref_->transform_cloud(*rcv_cloud, *pc2_ptr, "map")) {
            return nullptr;
        }
    }

    std::shared_ptr<msg::PointCloud> pc_ptr = std::make_shared<msg::PointCloud>();
    if (!sensor_msgs::convertPointCloud2ToPointCloud(*pc2_ptr, *pc_ptr)) {
        return nullptr;
    }

    return pc_ptr;
}

void PointCloudLayer::PointCloudCallback(msg::PointCloud2Ptr rcv_cloud) {

    const double callback_start_time = node_ref_->get_now_seconds();

    std::shared_ptr<msg::PointCloud> pc = RegisterPc2Msg(rcv_cloud);

    if (pc == nullptr) {
        return;
    }

    const std::string veh_frame = current_odom_.child_frame_id;

    if (veh_frame.empty()) {
        return;
    }

    if (clr_only_pc_ != nullptr) {
        msg::PoseStamped origin_pose = node_ref_->lookup_pose("map", veh_frame, clr_only_pc_->header.stamp);
        ClearPoints(clr_only_pc_, origin_pose.pose);
        clr_only_pc_ = nullptr;
    }

    msg::PoseStamped origin_pose = node_ref_->lookup_pose("map", veh_frame, rcv_cloud->header.stamp);
    ProcessPoints(pc, origin_pose.pose);

    pc_callback_time_.AddSample(node_ref_->get_now_seconds() - callback_start_time);
    if (pc_callback_time_.GetMean() > pc_callback_warn_dur_) {
        node_ref_->log_warning_throttle(1.0, "PointCloudCallback took %.2f ms (> %.2f ms warning threshold).",
            pc_callback_time_.GetMean()*1e3,
            pc_callback_warn_dur_*1e3
            );
    }
}

void PointCloudLayer::ClearOnlyPointsCallback(msg::PointCloud2Ptr rcv_cloud) {
    clr_only_pc_ = RegisterPc2Msg(rcv_cloud);
}


void PointCloudLayer::SetupPointCloudFilter() {

    PointCloudFilterConfig filter_pc_config = ParseFilterConfig();
    PointCloudFilterConfig filter_pc_cm_config = ParseFilterConfig("clear_method_");

    pc_filter.SetConfig(filter_pc_config);
    pc_cm_filter.SetConfig(filter_pc_cm_config);

    node_ref_->log_info(
        "Point cloud culling: %s",
        pc_filter.GetDescription().c_str()
        );

    node_ref_->log_info(
        "Point cloud culling (extra for grid clearing): %s",
        pc_cm_filter.GetDescription().c_str()
        );
}

void PointCloudLayer::ProcessPoints(const std::shared_ptr<msg::PointCloud>& pc_ptr, const msg::Pose& vehicle_pose, const bool clear_only) {

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

    AddOccupancy(*filtered_pc, cells_, dilation_.enabled);
    for (auto& cm : clear_methods_) {
        cm->OnOccupancyAdded(*filtered_cms_pc, vehicle_pose.position);
    }

}

void PointCloudLayer::AddOccupancy(const msg::PointCloud& point_cloud, std::vector< std::vector<Cell> >& cells, bool dilate) {

    bool has_segmentation_local = !point_cloud.channels.empty() && point_cloud.channels[0].name == pc_seg_channel_;
    has_segmentation_ = has_segmentation_local || has_segmentation_;

    const auto llx = size_info_.llx;
    const auto lly = size_info_.lly;
    const auto res = size_info_.res;
    const auto nx = size_info_.nx();
    const auto ny = size_info_.ny();

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

void PointCloudLayer::ClearPoints(const std::shared_ptr<msg::PointCloud>& pc_ptr, const msg::Pose& vehicle_pose) {
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

void PointCloudLayer::SetupGridClearingMethod() {

    ClearMethodRosParameters params = ParseClearMethodsConfig();

    BaseClearingSettings base_config;
    base_config.llx = size_info_.llx;
    base_config.lly = size_info_.lly;
    base_config.res = size_info_.res;
    base_config.grid_dilate_x = dilation_.GetNx(size_info_.res);
    base_config.grid_dilate_y = dilation_.GetNy(size_info_.res);
    base_config.thresh = thresholds_.thresh;
    base_config.immediate_clear_dilation = params.immediate_clr_dilation;
    base_config.visualization_range = params.visualization_range;
    base_config.visualize = params.visualize;

    clear_methods_ = ClearingMethodFactory::CreateClearingMethods(node_ref_, cells_, params, base_config, this);
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
