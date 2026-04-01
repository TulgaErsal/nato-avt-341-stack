#include "avt_341/perception/layers/point_cloud_layer.h"

namespace avt_341::perception
{

void PointCloudLayer::SetPointCloudFilterConfig(
        const PointCloudFilterConfig& filter_pc_config,
        const PointCloudFilterConfig& filter_pc_cm_config) {

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

    AddOccupancy(*filtered_pc, cells_, dilate_);
    for (auto& cm : clear_methods_) {
        cm->OnOccupancyAdded(*filtered_cms_pc, vehicle_pose.position);
    }

}

void PointCloudLayer::AddOccupancy(const avt_341::msg::PointCloud& point_cloud, std::vector< std::vector<Cell> >& cells, bool dilate) {

    bool has_segmentation_local = !point_cloud.channels.empty() && point_cloud.channels[0].name == "segmentation";
    has_segmentation_ = has_segmentation_local || has_segmentation_;

    const int dsize_x = GetDilateXSize();
    const int dsize_y = GetDilateYSize();

    // fill the cells with highest and lowest points
    for (int i = 0; i < point_cloud.points.size(); i++) {
        if (!(point_cloud.points[i].x == 0.0 && point_cloud.points[i].y == 0.0)) {
            int xi = (int)floor((point_cloud.points[i].x - llx_) / res_);
            int yi = (int)floor((point_cloud.points[i].y - lly_) / res_);
            if (xi >= 0 && xi < nx_ && yi >= 0 && yi < ny_) {
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
                    float terr_val = point_cloud.channels[0].values[i];
                    cell.terrain = fmax(cell.terrain, terr_val);
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
                    DilateCell(cells, xi, yi, dsize_x, dsize_y, original_slope);
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
    	ClearGrid();
    	for (auto& cm : clear_methods_) {
    		cm->Reset();
    	}
    	std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    is_resetting_ = false;
}

void PointCloudLayer::SetGridClearingMethod(const ClearMethodRosParameters & params) {

    BaseClearingSettings base_config;
    base_config.size_info = &size_info_;
    base_config.thresholds = &thresholds_;
    base_config.dilation = &dilation_;
    base_config.immediate_clear_dilation = params.immediate_clr_dilation;
    base_config.visualization_range = params.visualization_range;
    base_config.visualize = params.visualize;

    clear_methods_ = ClearingMethodFactory::CreateClearingMethods(node_ref_, cells_, params, base_config, this);
}

}