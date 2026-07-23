
#ifndef AVT_341_COSTMAP_LAYER_H
#define AVT_341_COSTMAP_LAYER_H
#include "avt_341/core/compute_time_recorder.hpp"
#include "avt_341/core/grid_components.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/perception/costmap_dtos.h"
#include "avt_341/perception/perception_settings.hpp"

namespace avt_341::perception
{
	class CostmapLayer
	{
	public:

		CostmapLayer(
			const std::shared_ptr<node::NodeProxy>& node_ref,
			const PerceptionSettings& settings,
			const std::string& label,
			const std::shared_ptr<core::ComputeTimeRecorder>& compute_time_recorder,
			bool contribute_occupancy,
			bool contribute_segmentation
		);

		virtual ~CostmapLayer() = default;

		bool HasSegmentation() const { return has_segmentation_; }
		bool ContributeOccupancy() const { return contribute_occupancy_; }
		bool ContributeSegmentation() const { return contribute_segmentation_; }

		Cell& CellAt(const int x, const int y) { return cells_[y][x]; }

		int GetGridCellValue(const Cell& cell) const;

		/// x and y in local ENU meters
		inline float GetRmsAtCoordinate(float x, float y) const;
		/// xi and yi as grid cell indices
		float GetRmsAtCell(int xi, int yi) const { return cells_[yi][xi].rms; }

		/// x and y in local ENU meters
		float GetTerrainSlopeAtCoordinate(float x, float y);
		/// xi and yi as grid cell indices
		float GetTerrainSlopeAtCell(int xi, int yi);

		virtual void RecomputeGridDilation();
		void UpdateThresholds(float slope_threshold, float slope_threshold_max);

		virtual bool PastSlopeThreshold(const Cell& cell) const {
			return Slope(cell) > settings_.costmap.thresholds.thresh;
		}
		virtual float Slope(const Cell& cell) const {
			return cell.height() / settings_.size_info().res;
		}

		virtual void Reset();
		bool IsEnabled() const { return is_enabled_; }
		bool HasData() const;
		void Resize();
		virtual void Clear();
		virtual void Visualize();

		const core::GridRegion& GetUpdateRegion() const { return grid_update_region_; }
		void ResetUpdateRegion() { grid_update_region_.Reset(); }
		void UpdateOdometry(const msg::Odometry& odom_msg);

		int GetSegValue(const int i, const int j) const { return cells_[i][j].terrain_seg; }
		int GetOccValue(const int i, const int j) const { return std::max(GetGridCellValue(cells_[i][j]), cells_[i][j].dilated_val); }
		std::string GetLabel() const { return label_; }

		virtual std::string ToString() const { return label_; }

	protected:

		void DilateCell(std::vector<std::vector<Cell>>& cells, int xi, int yi, float original_slope = 0.0f);

		std::vector< std::vector<Cell> > cells_;
		bool has_segmentation_ = false;
		bool is_resetting_ = false;
		bool is_enabled_ = true;
		bool contribute_occupancy_ = true;
		bool contribute_segmentation_ = true;

		std::shared_ptr<node::NodeProxy> node_ref_;
		std::shared_ptr<core::ComputeTimeRecorder> compute_time_recorder_;
		PerceptionSettings settings_;
		std::string label_;

		msg::Odometry current_odom_;

		core::GridRegion grid_update_region_;
	};
}
#endif //AVT_341_COSTMAP_LAYER_H
