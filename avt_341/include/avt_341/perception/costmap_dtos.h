#ifndef AVT_341_COSTMAP_CELL_H
#define AVT_341_COSTMAP_CELL_H

#include <vector>
#include <limits>
#include <avt_341/node/ros_types.h>
#include <string>
#include "avt_341/avt_341_utils.h"
#include <optional>

namespace avt_341::perception
{
  constexpr uint8_t GRID_MAX_VALUE = 100;

  class ElevAge{
  public:
    ElevAge(){
      val = 0.0f;
      age = 0.0f;
    }
    float val;
    float age;
  };

  class Cell{

    constexpr static const float MIN_LIMIT = std::numeric_limits<float>::lowest();
    constexpr static const float MAX_LIMIT = std::numeric_limits<float>::max();

  public:
    Cell(){
      low.val = MAX_LIMIT;
      high.val = MIN_LIMIT;
      has_dilated = false;
      dilated_val = -1;
      terrain_seg = -1;

      // RMS Statistics
      num_points = 0;
      summed_elev = 0.0f;
      avg_elev = 0.0f;
      rms = -1.0f;
      sum_of_squares = 0.0f;
    }

    static Cell Empty(){ return Cell(); }

    void AgeCell(float dt){
      low.age += dt;
      high.age += dt;
    }

    void ResetHeight(){
      // Need to keep dilated_val since from other adjacent cell
      low.val = MAX_LIMIT;
      high.val = MIN_LIMIT;
    }


    ElevAge low,high;

    float height() const { return high.val - low.val; }
    bool filled() const { return low.val < MAX_LIMIT; }

    bool has_dilated;
    int dilated_val;
    int terrain_seg;

    // RMS Statistics
    int num_points;
    float summed_elev;
    float avg_elev;
    float rms;
    float sum_of_squares;
  };

  class CellObstacleCalculator {
  public:
    virtual void AddOccupancy(const msg::PointCloud &point_cloud, std::vector< std::vector<Cell> > & cells, bool dilate) = 0;
    virtual bool PastSlopeThreshold(const Cell & cell) const = 0;
    virtual float Slope(const Cell & cell) const = 0;
  };

  struct TerrainRmsSettings
  {
    float hfov;
    float range;
    float time_window;
    int n_window;

    void SetDiscreteRmsWindow(float perception_rate)
    {
      n_window = static_cast<int>(time_window * perception_rate);
    }
  };

  struct GridPubMethod {
    static constexpr std::string_view Full = "full";
    static constexpr std::string_view Window = "window";
    static constexpr std::string_view Updates = "updates";

    static bool IsValid(const std::string & selected_method){
      const auto valid_methods = {Full, Window, Updates};
      return std::find(valid_methods.begin(), valid_methods.end(), selected_method) != valid_methods.end();
    }

  };

  struct LayerCombinationMethod {
    static constexpr std::string_view Last = "last";
    static constexpr std::string_view Mean = "mean";
    static constexpr std::string_view Max = "max";

    static bool IsLast(const std::string &method){
      return method == Last;
    }

    static bool IsMean(const std::string &method){
      return method == Mean;
    }

    static bool IsMax(const std::string &method){
      return method == Max;
    }

    static bool IsValid(const std::string & method){
      const auto valid_methods = {Last, Mean, Max};
      return std::find(valid_methods.begin(), valid_methods.end(), method) != valid_methods.end();
    }

    static void SetFlags(const std::string & method, bool & last_flag, bool & mean_flag){
      if (!IsValid(method))
      {
        throw std::invalid_argument("Invalid layer combination method: " + method);
      }
      last_flag = IsLast(method);
      mean_flag = IsMean(method);
    }

  };

  struct ThresholdSettings
  {
    bool use_elevation;
    float thresh;
    float thresh_max;
    float grid_slope_mult() const { return static_cast<float>(GRID_MAX_VALUE) / (thresh_max - thresh);}

    void Update(
      const std::optional<float> tr,
      const std::optional<float> tr_max)
    {
      thresh = std::max(0.0f, tr.value_or(thresh));
      thresh_max = std::max(tr_max.value_or(thresh_max), thresh);

      constexpr float eps = std::numeric_limits<float>::epsilon();
      if (std::abs(thresh_max - thresh) < eps) {
        thresh_max = thresh + eps;
      }
    }

    std::string ToString() const
    {
      return "use_elevation: " + std::to_string(use_elevation) + ", thresh: " + std::to_string(thresh) + ", thresh_max: " + std::to_string(thresh_max);
    }
  };

  struct DilationSettings
  {
    bool enabled;
    float x;
    float y;
    float proportion;

    int GetNx(const float res) const { return enabled ? lround(x/res) : 0;}
    int GetNy(const float res) const { return enabled ? lround(y/res) : 0;}

    std::string ToString() const
    {
      if (!enabled) {
        return "disabled";
      }

      return "x: " + std::to_string(x) + "m, y: " + std::to_string(y) + "m, proportion: " + std::to_string(proportion);
    }

  };

  struct CostmapSizeInfo
  {
    float width;
    float height;
    float res;
    float llx;
    float lly;

    int nx() const { return static_cast<int>(ceil(width / res));}
    int ny() const { return static_cast<int>(ceil(height / res));}

    utils::vec2 ToPosWorld(const int idxX, const int idxY) const {
      return utils::vec2(ToXWorld(idxX), ToYWorld(idxY));
    }

    utils::ivec2 ToIdx(const float x, const float y) const {
      return utils::ivec2(ToXIdx(x), ToYIdx(y));
    }

    float ToXWorld(const int i, const float offset=0.5f) const { return (static_cast<float>(i) + offset) * res + llx; }
    float ToYWorld(const int j, const float offset=0.5) const { return (static_cast<float>(j) + offset) * res + lly; }

    float ToXWorldLlc(const int i) const { return ToXWorld(i, 0.0f); }
    float ToYWorldLlc(const int j) const { return ToYWorld(j, 0.0f); }

    float ToXIdxFlt(const float x) const { return (x - llx) / res; }
    float ToYIdxFlt(const float y) const { return (y - lly) / res; }

    int ToXIdx(const float x) const { return static_cast<int>(ToXIdxFlt(x)); }
    int ToYIdx(const float y) const { return static_cast<int>(ToYIdxFlt(y)); }

    nav_msgs::msg::MapMetaData ToRosMetadata() const {
      nav_msgs::msg::MapMetaData meta;
      meta.resolution = res;
      meta.width = nx();
      meta.height = ny();
      meta.origin.position.x = llx;
      meta.origin.position.y = lly;
      meta.origin.orientation.w = 1.0;
      meta.origin.orientation.x = 0.0;
      meta.origin.orientation.y = 0.0;
      meta.origin.orientation.z = 0.0;
      return meta;
    }

    std::string ToString() const
    {
      return std::to_string(width) + "x" + std::to_string(height) + "m " + std::to_string(res) + "res (" + std::to_string(nx()) + "x" + std::to_string(ny()) + " cells), origin: " + std::to_string(llx) + "m , " + std::to_string(lly) + "m";
    }

  };

  struct CostmapSettings
  {
    CostmapSettings(
      const CostmapSizeInfo& size_info,
      const ThresholdSettings& thresholds,
      const DilationSettings& dilation,
      const TerrainRmsSettings& terrain_rms
      )
    : size_info(size_info), thresholds(thresholds), dilation(dilation), terrain_rms(terrain_rms)
    {
    }

    CostmapSizeInfo size_info;
    ThresholdSettings thresholds;
    DilationSettings dilation;
    TerrainRmsSettings terrain_rms;
  };

  struct PolygonZone
  {
    std::string label;
    std::vector<utils::vec2> vertices;
    double occ_value;
    int seg_value;
  };


}

#endif //AVT_341_COSTMAP_CELL_H
