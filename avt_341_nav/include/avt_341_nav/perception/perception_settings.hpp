#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>

#include <avt_341_nav/costmap_geometry_mixin_params_dto.hpp>
#include <avt_341_nav/perception_params_dto.hpp>

#include "avt_341_nav/core/math_dto.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"

namespace avt_341_nav::perception {

using GeneratedPerceptionParams = avt_341_nav::params::perception::Params;

/**
 * Generated perception parameters extended with costmap-domain operations.
 *
 * This class intentionally adds no data members. Computed values are evaluated
 * from the current generated parameter snapshot, so assigning a new Params
 * base snapshot cannot leave derived state stale.
 */
class PerceptionSettings final : public GeneratedPerceptionParams {
public:
  using Base = GeneratedPerceptionParams;
  // shared costmap geometry class from the costmap_info mixin DTO
  using SizeInfo = avt_341_nav::params::core::Geometry;

  PerceptionSettings() = default;
  explicit PerceptionSettings(Base params) : Base(std::move(params)) {}

  PerceptionSettings(const PerceptionSettings&) = default;
  PerceptionSettings(PerceptionSettings&&) = default;
  PerceptionSettings& operator=(const PerceptionSettings&) = default;
  PerceptionSettings& operator=(PerceptionSettings&&) = default;

  PerceptionSettings& operator=(Base params) {
    static_cast<Base&>(*this) = std::move(params);
    return *this;
  }

  [[nodiscard]] const SizeInfo& size_info() const {
    return costmap.geometry;
  }

  [[nodiscard]] static int nx(const SizeInfo& size) {
    return static_cast<int>(std::ceil(size.width / size.res));
  }

  [[nodiscard]] static int ny(const SizeInfo& size) {
    return static_cast<int>(std::ceil(size.height / size.res));
  }

  [[nodiscard]] int nx() const { return nx(size_info()); }
  [[nodiscard]] int ny() const { return ny(size_info()); }

  [[nodiscard]] static float to_x_world(
      const SizeInfo& size, const int index, const float offset = 0.5F) {
    return (static_cast<float>(index) + offset) * size.res + size.llx;
  }

  [[nodiscard]] static float to_y_world(
      const SizeInfo& size, const int index, const float offset = 0.5F) {
    return (static_cast<float>(index) + offset) * size.res + size.lly;
  }

  [[nodiscard]] float to_x_world(
      const int index, const float offset = 0.5F) const {
    return to_x_world(size_info(), index, offset);
  }

  [[nodiscard]] float to_y_world(
      const int index, const float offset = 0.5F) const {
    return to_y_world(size_info(), index, offset);
  }

  [[nodiscard]] static float to_x_index(
      const SizeInfo& size, const float x) {
    return (x - size.llx) / size.res;
  }

  [[nodiscard]] static float to_y_index(
      const SizeInfo& size, const float y) {
    return (y - size.lly) / size.res;
  }

  [[nodiscard]] static int to_x_index_int(
      const SizeInfo& size, const float x) {
    return static_cast<int>(to_x_index(size, x));
  }

  [[nodiscard]] static int to_y_index_int(
      const SizeInfo& size, const float y) {
    return static_cast<int>(to_y_index(size, y));
  }

  [[nodiscard]] int to_x_index(const float x) const {
    return to_x_index_int(size_info(), x);
  }

  [[nodiscard]] int to_y_index(const float y) const {
    return to_y_index_int(size_info(), y);
  }

  [[nodiscard]] core::ivec2 to_index(
      const float x, const float y) const {
    return core::ivec2(to_x_index(x), to_y_index(y));
  }

  [[nodiscard]] core::vec2 to_world(
      const int x_index, const int y_index) const {
    return core::vec2(to_x_world(x_index), to_y_world(y_index));
  }

  [[nodiscard]] int dilation_x_cells() const {
    return costmap.dilation.enabled
               ? static_cast<int>(
                     std::lround(costmap.dilation.x / size_info().res))
               : 0;
  }

  [[nodiscard]] int dilation_y_cells() const {
    return costmap.dilation.enabled
               ? static_cast<int>(
                     std::lround(costmap.dilation.y / size_info().res))
               : 0;
  }

  [[nodiscard]] float grid_slope_multiplier() const {
    return 100.0F /
           (costmap.thresholds.thresh_max - costmap.thresholds.thresh);
  }

  [[nodiscard]] int rms_window_samples() const {
    return static_cast<int>(
        costmap.terrain_rms.time_window * runtime.rate);
  }

  void update_thresholds(
      const std::optional<float> threshold,
      const std::optional<float> maximum) {
    auto& values = costmap.thresholds;
    values.thresh = std::max(0.0F, threshold.value_or(values.thresh));
    values.thresh_max =
        std::max(maximum.value_or(values.thresh_max), values.thresh);

    constexpr float epsilon = std::numeric_limits<float>::epsilon();
    if (std::abs(values.thresh_max - values.thresh) < epsilon) {
      values.thresh_max = values.thresh + epsilon;
    }
  }

  [[nodiscard]] nav_msgs::msg::MapMetaData to_ros_metadata() const {
    nav_msgs::msg::MapMetaData metadata;
    metadata.resolution = size_info().res;
    metadata.width = nx();
    metadata.height = ny();
    metadata.origin.position.x = size_info().llx;
    metadata.origin.position.y = size_info().lly;
    metadata.origin.orientation.w = 1.0;
    return metadata;
  }

  [[nodiscard]] std::string size_info_string() const {
    return std::to_string(size_info().width) + "x" +
           std::to_string(size_info().height) + "m " +
           std::to_string(size_info().res) + "res (" +
           std::to_string(nx()) + "x" + std::to_string(ny()) +
           " cells), origin: " + std::to_string(size_info().llx) +
           "m, " + std::to_string(size_info().lly) + "m";
  }

  [[nodiscard]] std::string thresholds_string() const {
    const auto& values = costmap.thresholds;
    return "use_elevation: " + std::to_string(values.use_elevation) +
           ", thresh: " + std::to_string(values.thresh) +
           ", thresh_max: " + std::to_string(values.thresh_max) +
           ", output_unknown_cells: " +
           std::to_string(values.output_unknown_cells) +
           ", replace_occ_unknown_with: " +
           std::to_string(values.replace_occ_unknown_with) +
           ", replace_seg_unknown_with: " +
           std::to_string(values.replace_seg_unknown_with);
  }

  [[nodiscard]] std::string dilation_string() const {
    const auto& values = costmap.dilation;
    if (!values.enabled) {
      return "disabled";
    }
    return "x: " + std::to_string(values.x) +
           "m, y: " + std::to_string(values.y) +
           "m, proportion: " + std::to_string(values.proportion);
  }
};

using CostmapSizeInfo = PerceptionSettings::SizeInfo;
using ClearMethodSettings = GeneratedPerceptionParams::ClearMethod;

}  // namespace avt_341_nav::perception
