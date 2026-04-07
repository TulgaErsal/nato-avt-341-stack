
#ifndef AVT_341_COSTMAP_LAYER_H
#define AVT_341_COSTMAP_LAYER_H
#include "avt_341/core/grid_components.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/perception/costmap_dtos.h"

namespace avt_341::perception
{
	class CostmapLayer
	{
	public:

		CostmapLayer(
			const std::shared_ptr<node::NodeProxy>& node_ref,
			const CostmapSettings& cm_settings,
			const std::string& label
		);

		bool HasSegmentation() const { return has_segmentation_; }

		Cell& CellAt(const int x, const int y) { return cells_[y][x]; }

		int GetGridCellValue(const Cell& cell) const;

		/// x and y in local ENU meters
		float GetRmsAtCoordinate(float x, float y);
		/// xi and yi as grid cell indices
		float GetRmsAtCell(int xi, int yi);

		/// x and y in local ENU meters
		float GetTerrainSlopeAtCoordinate(float x, float y);
		/// xi and yi as grid cell indices
		float GetTerrainSlopeAtCell(int xi, int yi);

		virtual void RecomputeGridDilation();

		virtual bool PastSlopeThreshold(const Cell& cell) const { return cell.height() / size_info_.res > thresholds_.thresh; }
		virtual float Slope(const Cell& cell) const { return cell.height() / size_info_.res; }

		virtual void Reset();
		bool IsValid() const { return is_valid_; }
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
		bool is_valid_ = true;

		std::shared_ptr<node::NodeProxy> node_ref_;
		CostmapSizeInfo size_info_;
		ThresholdSettings thresholds_;
		DilationSettings dilation_;
		std::string label_;

		msg::Odometry current_odom_;

		core::GridRegion grid_update_region_;
	};
}
#endif //AVT_341_COSTMAP_LAYER_H
