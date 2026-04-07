#include "avt_341/perception/layers/static_grid_layer.h"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>
#include <stdexcept>

namespace avt_341::perception
{
    StaticGridLayer::StaticGridLayer(
        const std::shared_ptr<node::NodeProxy>& node_ref,
        const CostmapSettings& cm_settings,
        const std::string& label
        )
        : CostmapLayer(node_ref, cm_settings, label)
    {
        // Format map_x_{x}_y_{y}_res_{res}_w_{w}_h_{h}.csv
        // Where x,y = map lower-left corner, res = resolution, w,h = width,height survey range in meters
        node_ref_->get_parameter("~static_grid_layer_data_file", input_file_, std::string(""));

        // Field in csv file to look for height values
        node_ref_->get_parameter("~static_grid_height_field", csv_height_field_, std::string("height"));

        // Field in csv file to look for segmentation values
        node_ref_->get_parameter("~static_grid_segmentation_field", csv_segmentation_field_, std::string("segmentation"));

        try
        {
            LoadStaticGrid();
        }
        catch(const std::exception& e)
        {
            node_ref_->log_error("Failed to load static grid layer: %s", e.what());
            is_valid_ = false;
        }

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
        info.res = 1.0f;
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

    void StaticGridLayer::LoadStaticGrid()
    {
        if (input_file_.empty()){
            is_valid_ = false;
            return;
        }

        if (!std::filesystem::exists(input_file_)){
            throw std::runtime_error("Input file does not exist: " + input_file_);
        }

        const CostmapSizeInfo file_info = ParseSizeInfoFromFile(input_file_);
        int file_nx = file_info.nx();
        int file_ny = file_info.ny();

        // Read CSV data using header fields to locate height and segmentation columns
        std::vector<float> file_heights(file_nx * file_ny, 0.0f);
        std::vector<int>   file_segs(file_nx * file_ny, -1);
        {
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
                node_ref_->log_warning(
                    "[StaticGridLayer] '%s': height field '%s' not found in CSV header. Layer is invalid.",
                    label_.c_str(), csv_height_field_.c_str());
                is_valid_ = false;
                return;
            }

            has_segmentation_ = seg_col != -1;
            node_ref_->log_info("[StaticGridLayer] '%s': segmentation data found: %d.",
                label_.c_str(), has_segmentation_);

            // Read data rows using discovered column indices
            std::string line;
            int idx = 0;
            while (std::getline(ifs, line) && idx < file_nx * file_ny)
            {
                std::istringstream ss(line);
                std::string token;
                std::vector<std::string> row_fields;
                while (std::getline(ss, token, ',')) row_fields.push_back(token);

                file_heights[idx] = std::stof(row_fields[height_col]);
                if (has_segmentation_)
                {
                    file_segs[idx] = static_cast<int>(std::stof(row_fields[seg_col]));
                }

                // if (height_col < static_cast<int>(row_fields.size()))
                // {
                //
                //     try { file_heights[idx] = std::stof(row_fields[height_col]); }
                //     catch (...) {}
                // }
                // if (seg_col != -1 && seg_col < static_cast<int>(row_fields.size()))
                // {
                //     try { file_segs[idx] = static_cast<int>(std::stof(row_fields[seg_col])); }
                //     catch (...) {}
                // }
                idx++;
            }
        }

        // Layer grid info
        const float layer_res = size_info_.res;
        const int   layer_nx  = size_info_.nx();
        const int   layer_ny  = size_info_.ny();

        // Ratio: how many file cells per layer cell
        float ratio = layer_res / file_info.res;

        if (ratio < 1.0f){
            throw std::runtime_error("layer resolution =" + std::to_string(layer_res) + " < file resolution = " + std::to_string(file_info.res));
        }

        for (int yi = 0; yi < layer_ny; yi++) {
            for (int xi = 0; xi < layer_nx; xi++) {
                // Corresponding file cell indices (lower-left of sub-region)
                float fx0f = (size_info_.ToXWorld(xi) - file_info.llx) / file_info.res;
                float fy0f = (size_info_.ToYWorld(yi) - file_info.lly) / file_info.res;
                fy0f = static_cast<float>(file_ny) - 1.0f - fy0f; // flip y-axis since file origin is top-left

                int fx0 = static_cast<int>(std::floor(fx0f));
                int fy0 = static_cast<int>(std::floor(fy0f));
                int fx1 = static_cast<int>(std::ceil(fx0f + ratio)) - 1;
                int fy1 = static_cast<int>(std::ceil(fy0f + ratio)) - 1;

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
                        float h = file_heights[fidx];
                        h_min = std::min(h_min, h);
                        h_max = std::max(h_max, h);
                        if (seg_val == -1) seg_val = file_segs[fidx];
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

    void StaticGridLayer::Clear()
    {
        // No need to clear layer since has no internal changing state
    }
}

