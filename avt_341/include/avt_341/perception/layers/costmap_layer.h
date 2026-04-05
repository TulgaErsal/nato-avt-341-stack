
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
			const CostmapSizeInfo& size_info,
			const ThresholdSettings& thresholds,
			const DilationSettings& dilation
		);

		bool HasSegmentation() const { return has_segmentation_; }

		Cell& CellAt(const int x, const int y) { return cells_[y][x]; }
		std::vector<Cell>& (&operator[](const int idx)) { return cells_[idx]; }

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

		inline bool PastSlopeThreshold(const Cell& cell) const;
		inline float Slope(const Cell& cell) const;

		virtual void Reset();
		bool IsValid() const { return is_valid_; }
		bool HasData() const;
		void Resize();
		void Clear();
		virtual void Visualize();

		const core::GridRegion& GetUpdateRegion() const { return grid_update_region_; }
		void ResetUpdateRegion() { grid_update_region_.Reset(); }
		void UpdateOdometry(const msg::Odometry& odom_msg);

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
		msg::Odometry current_odom_;

		core::GridRegion grid_update_region_;
	};
}
#endif //AVT_341_COSTMAP_LAYER_H
