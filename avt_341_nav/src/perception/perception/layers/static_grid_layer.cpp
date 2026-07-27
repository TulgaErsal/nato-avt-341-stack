#include "avt_341_nav/perception/layers/static_grid_layer.h"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>
#include <stdexcept>

namespace avt_341_nav::perception
{
    StaticGridLayer::StaticGridLayer(
        const rclcpp::Node::SharedPtr& node_ref,
        const PerceptionSettings& settings,
        const std::string& label,
        const std::shared_ptr<core::ComputeTimeRecorder>& compute_time_recorder,
        const avt_341_nav::params::perception::Params::StaticGridLayer& params
        )
        : CostmapLayer(
            node_ref, settings, label, compute_time_recorder,
            params.contribute_occupancy, params.contribute_segmentation),
          input_file_(params.data_file),
          csv_height_field_(params.height_field),
          csv_segmentation_field_(params.segmentation_field),
          input_y_dir_negative_(params.input_y_dir_negative)
    {
        LoadFileData();
    }

    std::string StaticGridLayer::ToString() const
    {
        return "[StaticGridLayer] id: " + label_
            + ", file: " + input_file_;
    }

    CostmapSizeInfo StaticGridLayer::ParseSizeInfoFromFile(const std::string& file_name)
    {
        // Extract just the filename portion (after last slash/backslash)
        std::string fname = file_name;
        auto slash_pos = fname.find_last_of("/\\");
        if (slash_pos != std::string::npos) fname = fname.substr(slash_pos + 1);
        // Remove .csv extension
        if (fname.size() > 4 && fname.substr(fname.size() - 4) == ".csv") {
            fname = fname.substr(0, fname.size() - 4);
        }

        CostmapSizeInfo info{};
        info.width = 0.0F;
        info.height = 0.0F;
        info.res = 1.0f;
        info.llx = 0.0F;
        info.lly = 0.0F;
        // Parse tokens separated by '_'
        // Format: x_{x}_y_{y}_res_{res}_w_{w}_h_{h}
        std::istringstream ss(fname);
        std::string token;
        std::vector<std::string> tokens;
        while (std::getline(ss, token, '_')) tokens.push_back(token);
        for (int i = 0; i + 1 < (int)tokens.size(); i++) {
            if (tokens[i] == "x")        info.llx    = std::stof(tokens[i+1]);
            else if (tokens[i] == "y")   info.lly    = std::stof(tokens[i+1]);
            else if (tokens[i] == "res") info.res    = std::stof(tokens[i+1]);
            else if (tokens[i] == "w")   info.width  = std::stof(tokens[i+1]);
            else if (tokens[i] == "h")   info.height = std::stof(tokens[i+1]);
        }
        return info;
    }

    void StaticGridLayer::ParseFileData(const CostmapSizeInfo& file_info, std::vector<float>& file_heights, std::vector<int>& file_segs)
    {
        int file_nx = PerceptionSettings::nx(file_info);
        int file_ny = PerceptionSettings::ny(file_info);

        // Read CSV data using header fields to locate height and segmentation columns
        file_heights.resize(file_nx * file_ny, 0.0f);
        file_segs.resize(file_nx * file_ny, -1);

        std::ifstream ifs(input_file_);
        if (!ifs.is_open()) throw std::runtime_error("Failed to open input_file_: " + input_file_);

        // Parse header row to find column indices
        std::string header_line;
        if (!std::getline(ifs, header_line))
            throw std::runtime_error("CSV file is empty: " + input_file_);

        std::vector<std::string> header_fields;
        {
            std::istringstream hss(header_line);
            std::string field;
            while (std::getline(hss, field, ','))
            {
                // Trim whitespace
                field.erase(0, field.find_first_not_of(" \t\r\n"));
                field.erase(field.find_last_not_of(" \t\r\n") + 1);
                header_fields.push_back(field);
            }
        }

        int height_col = -1;
        int seg_col    = -1;
        for (int i = 0; i < static_cast<int>(header_fields.size()); ++i)
        {
            if (header_fields[i] == csv_height_field_)       height_col = i;
            else if (header_fields[i] == csv_segmentation_field_) seg_col = i;
        }

        if (height_col == -1)
        {
            RCLCPP_WARN(node_ref_->get_logger(), "[StaticGridLayer] '%s': height field '%s' not found in CSV header. Layer is invalid.", label_.c_str(), csv_height_field_.c_str());
            is_enabled_ = false;
            return;
        }

        has_segmentation_ = seg_col != -1;
        RCLCPP_INFO(node_ref_->get_logger(), "[StaticGridLayer] '%s': segmentation data found: %d.", label_.c_str(), has_segmentation_);

        // Read data rows using discovered column indices
        std::string line;
        int idx = 0;
        while (std::getline(ifs, line) && idx < file_nx * file_ny)
        {
            std::istringstream ss(line);
            std::string token;
            std::vector<std::string> row_fields;
            while (std::getline(ss, token, ',')) row_fields.push_back(token);

            auto data_idx = idx;
            if (input_y_dir_negative_)
            {
                const auto y_idx_flip = file_ny - 1 - idx / file_nx;      // file_ny - 1 - y_idx_in
                data_idx = y_idx_flip * file_nx + (idx % file_nx);           // y_idx_flip * file_nx + x_idx_in
            }

            file_heights[data_idx] = std::stof(row_fields[height_col]);
            if (has_segmentation_)
            {
                file_segs[data_idx] = static_cast<int>(std::stof(row_fields[seg_col]));
            }
            idx++;
        }

        if (idx != file_nx * file_ny){
            throw std::runtime_error("CSV file has " + std::to_string(idx) + " data rows, but expected " + std::to_string(file_nx * file_ny) + " based on size info.");
        }
    }

    void StaticGridLayer::SetStaticData(const CostmapSizeInfo& size_info_in, const std::vector<float>& heights_in, const std::vector<int>& segs_in)
    {
        const int file_nx = PerceptionSettings::nx(size_info_in);
        const int file_ny = PerceptionSettings::ny(size_info_in);

        // Layer grid info
        const float layer_res = settings_.size_info().res;
        const int   layer_nx  = settings_.nx();
        const int   layer_ny  = settings_.ny();

        // Ratio: how many file cells per layer cell
        float ratio = layer_res / size_info_in.res;

        if (ratio < 1.0f){
            throw std::runtime_error("layer resolution =" + std::to_string(layer_res) + " < file resolution = " + std::to_string(size_info_in.res));
        }

        for (int yi = 0; yi < layer_ny; yi++) {
            for (int xi = 0; xi < layer_nx; xi++) {
                // Corresponding file cell indices (lower-left of sub-region)
                float fx0f = PerceptionSettings::to_x_index(
                    size_info_in, settings_.to_x_world(xi, 0.0F));
                float fy0f = PerceptionSettings::to_y_index(
                    size_info_in, settings_.to_y_world(yi, 0.0F));

                int fx0 = static_cast<int>(std::floor(fx0f));
                int fy0 = static_cast<int>(std::floor(fy0f));
                int fx1 = static_cast<int>(std::floor(fx0f + ratio)) - 1;
                int fy1 = static_cast<int>(std::floor(fy0f + ratio)) - 1;

                // Clamp to file bounds
                fx0 = std::max(fx0, 0);
                fy0 = std::max(fy0, 0);
                fx1 = std::min(fx1, file_nx - 1);
                fy1 = std::min(fy1, file_ny - 1);

                if (fx0 > fx1 || fy0 > fy1) continue; // out of file coverage

                float h_min = std::numeric_limits<float>::max();
                float h_max = std::numeric_limits<float>::lowest();
                int seg_val = -1;

                for (int fyi = fy0; fyi <= fy1; fyi++) {
                    for (int fxi = fx0; fxi <= fx1; fxi++) {
                        int fidx = fyi * file_nx + fxi;
                        float h = heights_in[fidx];
                        h_min = std::min(h_min, h);
                        h_max = std::max(h_max, h);
                        if (seg_val == -1) seg_val = segs_in[fidx];
                    }
                }

                if (h_min <= h_max) {
                    cells_[yi][xi].low.val  = h_min;
                    cells_[yi][xi].high.val = h_max;
                    cells_[yi][xi].terrain_seg = seg_val;
                }
            }
        }
    }

    void StaticGridLayer::LoadFileData()
    {
        try
        {
            if (input_file_.empty()){
                is_enabled_ = false;
                return;
            }

            if (!std::filesystem::exists(input_file_)){
                throw std::runtime_error("Input file does not exist: " + input_file_);
            }

            const CostmapSizeInfo file_info = ParseSizeInfoFromFile(input_file_);

            std::vector<float> file_heights;
            std::vector<int> file_segs;
            ParseFileData(file_info, file_heights, file_segs);

            SetStaticData(file_info, file_heights, file_segs);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(node_ref_->get_logger(), "Failed to load static grid layer: %s", e.what());
            is_enabled_ = false;
        }
    }

    void StaticGridLayer::Clear()
    {
        // No need to clear layer since has no internal changing state
    }
}

