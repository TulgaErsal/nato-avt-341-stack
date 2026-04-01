
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

		std::vector<utils::ivec2> GetCellsInFov(float x, float y, float heading, float hfov, float range);
		uint8_t GetGridCellValue(const Cell& cell) const;

		/// x and y in local ENU meters
		float GetRmsAtCoordinate(float x, float y);
		/// xi and yi as grid cell indices
		float GetRmsAtCell(int xi, int yi);

		/// x and y in local ENU meters
		float GetTerrainSlopeAtCoordinate(float x, float y);
		/// xi and yi as grid cell indices
		float GetTerrainSlopeAtCell(int xi, int yi);

		void GetSlopeRmsInFov(float& slope, float& rms, float x, float y, float heading, float hfov, float range);
		virtual void RecomputeGridDilation();

		bool PastSlopeThreshold(const Cell& cell) const;
		float Slope(const Cell& cell) const;

		virtual void Reset();
		bool HasData() const;
		void Resize();
		void Clear();

	protected:

		void DilateCell(
			std::vector<std::vector<Cell>>& cells,
			int xi,
			int yi,
			int dsize_x,
			int dsize_y,
			float original_slope = 0.0f);

		int GetDilationNx() const { return dilation_.GetNx(size_info_.res); }
		int GetDilationNy() const { return dilation_.GetNy(size_info_.res); }

		std::vector< std::vector<Cell> > cells_;
		bool has_segmentation_ = false;
		bool is_resetting_ = false;

		std::shared_ptr<node::NodeProxy> node_ref_;
		CostmapSizeInfo size_info_;
		ThresholdSettings thresholds_;
		DilationSettings dilation_;

		core::GridRegion grid_update_region_;
	};
}
#endif //AVT_341_COSTMAP_LAYER_H
