
#ifndef AVT_341_STATIC_GRID_LAYER_H
#define AVT_341_STATIC_GRID_LAYER_H
#include "costmap_layer.h"
#include <avt_341/perception_params_dto.hpp>

namespace avt_341::perception
{
    class StaticGridLayer: public CostmapLayer
    {
    public:
        StaticGridLayer(
            const std::shared_ptr<node::NodeProxy>& node_ref,
            const PerceptionSettings& settings,
            const std::string& label,
            const std::shared_ptr<core::ComputeTimeRecorder>& compute_time_recorder,
            const avt_341::params::perception::Params::StaticGridLayer& params
            );

        std::string ToString() const override;
        void Clear() override;

        void LoadFileData();
        static CostmapSizeInfo ParseSizeInfoFromFile(const std::string& file_name);
        void SetStaticData(const CostmapSizeInfo& size_info_in, const std::vector<float>& heights_in, const std::vector<int>& segs_in);

    private:
        void ParseFileData(const CostmapSizeInfo& file_info, std::vector<float>& file_heights, std::vector<int>& file_segs);
        std::string input_file_;
        std::string csv_height_field_;
        std::string csv_segmentation_field_;
        bool input_y_dir_negative_;
    };
}


#endif //AVT_341_STATIC_GRID_LAYER_H
