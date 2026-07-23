#include "avt_341/perception/costmap.h"
#include "avt_341/perception/layers/point_cloud_layer.h"
#include "avt_341/perception/layers/polygon_layer.h"
#include "avt_341/perception/layers/static_grid_layer.h"
#include "avt_341/perception/layers/camera_layer.h"
#include <algorithm>

namespace avt_341::perception {

Costmap::Costmap(
	const std::shared_ptr<node::NodeProxy>& node_ref,
	const PerceptionSettings& settings
	)
	: node_ref_(node_ref),
	compute_time_recorder_(std::make_shared<core::ComputeTimeRecorder>(node_ref, core::ComputeTimeRecorder::MakeNodeTag(node_ref))),
	settings_(settings),
	layer_cmb_method_(settings.costmap.layer_combination_method)
{
	std::vector<std::shared_ptr<CostmapLayer>> candidate_layers = {
		std::make_shared<StaticGridLayer>(
			node_ref, settings, "static_grid_layer", compute_time_recorder_,
			settings.static_grid_layer),
		std::make_shared<PointCloudLayer>(
			node_ref, settings, "point_cloud_layer", compute_time_recorder_,
			settings.point_cloud_layer.topic,
			settings.point_cloud_layer.clear_topic,
			settings.point_cloud_layer.contribute_occupancy,
			settings.point_cloud_layer.contribute_segmentation),
		std::make_shared<CameraLayer>(
			node_ref, settings, "camera_layer", compute_time_recorder_,
			settings.camera_layer),
		std::make_shared<PolygonLayer>(
			node_ref, settings, "polygon_layer", compute_time_recorder_,
			settings.polygon_layer),
	};

	layers_.clear();
	std::copy_if(candidate_layers.begin(), candidate_layers.end(), std::back_inserter(layers_),
		[](const std::shared_ptr<CostmapLayer>& layer) {
		return layer->IsEnabled();
	});

	LayerCombinationMethod::SetFlags(layer_cmb_method_, layer_cmb_last_, layer_cmd_mn_);

	odom_sub_ = node_ref_->create_subscription<msg::Odometry>(
		"avt_341/odometry",
		10,
		std::bind(&Costmap::OdometryCallback, this, std::placeholders::_1));

}

void Costmap::UpdateThresholds(
	const float slope_threshold, const float slope_threshold_max)
{
	settings_.update_thresholds(slope_threshold, slope_threshold_max);
	for (const auto& layer : layers_) {
		layer->UpdateThresholds(slope_threshold, slope_threshold_max);
	}
}

void Costmap::OdometryCallback(msg::OdometryPtr rcv_odom) {
	current_odom_ = *rcv_odom;
	for (const auto & layer : layers_){
		layer->UpdateOdometry(current_odom_);
	}
}

bool Costmap::HasSegmentation() const
{
	return std::any_of(layers_.begin(), layers_.end(),
		[](const std::shared_ptr<CostmapLayer>& layer) { return layer->HasSegmentation(); }
	);
}

void Costmap::Clear() const
{
	for (const auto & layer : layers_) {
		layer->Clear();
	}
}

void Costmap::Reset() const
{
	for (const auto & layer : layers_) {
		layer->Reset();
	}
}

void Costmap::Visualize() const
{
	for (const auto & layer : layers_) {
		layer->Visualize();
	}
}

void Costmap::PublishComputeTimes() const
{
	compute_time_recorder_->PublishSummary();
}

std::vector<std::shared_ptr<CostmapLayer>> Costmap::GetTargetLayers(const std::string& target_layer, bool is_segmentation) const {

    std::vector<std::shared_ptr<CostmapLayer>> layers;
    if (target_layer.empty()){
        layers = layers_;
    }else{
        for (const auto & layer : layers_){
            if (layer->GetLabel() == target_layer){
                layers.push_back(layer);
                break;
            }
        }
    }

    layers.erase(std::remove_if(layers.begin(), layers.end(),
    [is_segmentation](const std::shared_ptr<CostmapLayer>& layer) {
        return is_segmentation ? !layer->ContributeSegmentation() : !layer->ContributeOccupancy();
    }), layers.end());

    return layers;
}

void Costmap::FillGridMsgCells(std::vector<int8_t> & data, const core::GridRegion region, bool is_segmentation, std::string target_layer) const {

	// TODO: Temporary change related to https://github.com/TulgaErsal/nato-avt-341-stack/issues/246
	//data.resize(region.Width()*region.Height());

	const auto layers = GetTargetLayers(target_layer, is_segmentation);

	const auto& thresholds = settings_.costmap.thresholds;
	const int unknown_value = thresholds.output_unknown_cells
		? -1
		: static_cast<int>(
		      is_segmentation ? thresholds.replace_seg_unknown_with
		                      : thresholds.replace_occ_unknown_with);

	int c = 0;
	for (int i = region.y_min; i < region.y_max; i++) {
		for (int j = region.x_min; j < region.x_max; j++) {
			const int layer_val = GetCombinedLayerValue<int>(layers, [i, j, is_segmentation](const std::shared_ptr<CostmapLayer>& layer){
				return is_segmentation ? layer->GetSegValue(i, j) : layer->GetOccValue(i, j);
			}, unknown_value);
			data[c++] = static_cast<int8_t>(layer_val);
		}
	}
}

msg::OccupancyGridUpdate Costmap::GetGridUpdate(
		const bool is_segmentation,
		const std::string& target_layer
	) const {
	msg::OccupancyGridUpdate grid_update_msg;
	grid_update_msg.header.frame_id = "map";

	core::GridRegion update_region;
	for (const auto & layer : GetTargetLayers(target_layer, is_segmentation)) {
		update_region.UpdateBounds(layer->GetUpdateRegion());
	}
	if (!update_region.HasData()) {
		return grid_update_msg;
	}
	const int dilate_x = settings_.dilation_x_cells();
	const int dilate_y = settings_.dilation_y_cells();
	core::GridRegion dilated_region = update_region.Dilate(
		dilate_x, dilate_y, settings_.nx(), settings_.ny());
	grid_update_msg.x = dilated_region.x_min;
	grid_update_msg.y = dilated_region.y_min;
	grid_update_msg.width = dilated_region.Width();
	grid_update_msg.height = dilated_region.Height();
	grid_update_msg.data.resize(dilated_region.Width()*dilated_region.Height());

	FillGridMsgCells(grid_update_msg.data, dilated_region, is_segmentation, target_layer);
	return grid_update_msg;
}

void Costmap::ResetUpdateRegion()
{
	for (const auto & layer : layers_) {
		layer->ResetUpdateRegion();
	}
}

msg::OccupancyGrid Costmap::GetGrid(
		const bool is_segmentation,
		const std::string& target_layer
	) const {
	msg::OccupancyGrid grid;
	grid.header.frame_id = "map";
	grid.info = settings_.to_ros_metadata();
	grid.data.resize(grid.info.width  * grid.info.height);
	const auto update_region = core::GridRegion(0, grid.info.width, 0, grid.info.height);

	FillGridMsgCells(grid.data, update_region, is_segmentation, target_layer);
	return grid;
}

msg::OccupancyGrid Costmap::GetGrid(double width, double height, bool is_segmentation) const {
	double local_x_origin = current_odom_.pose.pose.position.x - width / 2.0;
	double local_y_origin = current_odom_.pose.pose.position.y - height / 2.0;

	int local_nx = static_cast<int>(width / settings_.size_info().res);
	int local_ny = static_cast<int>(height / settings_.size_info().res);
	int xi_min = std::max(0, settings_.to_x_index(local_x_origin));
	int yi_min = std::max(0, settings_.to_y_index(local_y_origin));
	int xi_max = std::min(settings_.nx(), xi_min + local_nx);
	int yi_max = std::min(settings_.ny(), yi_min + local_ny);

	msg::OccupancyGrid grid;
	grid.header.frame_id = "map";
	grid.info = settings_.to_ros_metadata();
	grid.info.width = local_nx; //xi_max-xi_min;
	grid.info.height =  local_ny; //yi_max-yi_min;
	grid.info.origin.position.x = settings_.to_x_world(xi_min);
	grid.info.origin.position.y = settings_.to_y_world(yi_min);
	grid.data.resize(local_nx * local_ny);

	FillGridMsgCells(grid.data, core::GridRegion(xi_min, xi_max, yi_min, yi_max), is_segmentation);
	return grid;
}

bool Costmap::IsPointInCone(const utils::vec2& test_point, const utils::vec2& p, const utils::vec2& v, float r, float angle) {
	utils::vec2 dir_to_point = test_point - p;
	float dist = utils::length(dir_to_point);

	if (dist > r) return false;

	dir_to_point.normalize();
	float dot_product = utils::dot(v, dir_to_point);

	float cos_angle = cosf(angle);
	return dot_product >= cos_angle;
}

// CTG, 7/23/25
std::vector<utils::ivec2> Costmap::GetCellsInFov() const{

	float heading = utils::GetHeadingFromOrientation(current_odom_.pose.pose.orientation);
	const float x = current_odom_.pose.pose.position.x;
	const float y = current_odom_.pose.pose.position.y;

	utils::vec2 p(x, y);
	float angle = 0.5F * settings_.costmap.terrain_rms.hfov;
	utils::vec2 v(cosf(heading), sinf(heading));
	std::vector<utils::ivec2> cells_in_fov;
	const auto range = settings_.costmap.terrain_rms.range;

	int xi0 = std::max(
		0, static_cast<int>((x - range) / settings_.size_info().res));
	int xi1 = std::min(
		settings_.nx(),
		static_cast<int>((x + range) / settings_.size_info().res));
	int yi0 = std::max(
		0, static_cast<int>((y - range) / settings_.size_info().res));
	int yi1 = std::min(
		settings_.ny(),
		static_cast<int>((y + range) / settings_.size_info().res));
	for (int j = yi0; j < yi1; j++) {
		for (int i = xi0; i < xi1; i++) {
			utils::vec2 point = settings_.to_world(i, j);
			bool in_view = IsPointInCone(point, p, v, range, angle);
			if (in_view) {
				utils::ivec2 c(i, j);
				cells_in_fov.push_back(c);
			}
		}
	}
	return cells_in_fov;
}

// CTG, 7/23/25
void Costmap::UpdateRmsAndSlope() {

	std::vector<utils::ivec2> idxs = GetCellsInFov();
	float slope = 0.0f;
	float rms = 0.0f;

	if (idxs.empty()){
		return;
	}

	float navg = static_cast<float>(idxs.size());
	for (const auto & idx : idxs) {
		rms += GetCombinedLayerValue<float>(layers_, [&idx](const std::shared_ptr<CostmapLayer>& layer){
			return layer->GetRmsAtCell(idx.x, idx.y);
		});
		slope += GetCombinedLayerValue<float>(layers_, [&idx](const std::shared_ptr<CostmapLayer>& layer){
			return layer->GetTerrainSlopeAtCell(idx.x, idx.y);
		});
	}
	rms /= navg;
	slope /= navg;

	slope_buffer_.push_back(static_cast<double>(slope));
	rms_buffer_.push_back(static_cast<double>(rms));
	if (slope_buffer_.size() >
	    static_cast<std::size_t>(settings_.rms_window_samples())) {
		slope_buffer_.pop_front();
		rms_buffer_.pop_front();
	}
}

std::string Costmap::ToLayerInfoString() const
{
    std::string result = "Costmap layers (" + std::to_string(layers_.size()) + "), combination method: " + layer_cmb_method_ + "\n";
    for (const auto& layer : layers_) {
        result += "  - " + layer->ToString() + "\n";
    }
    return result;
}

} //namespace avt_341::perception
