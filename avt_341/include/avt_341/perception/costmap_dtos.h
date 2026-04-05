#ifndef AVT_341_COSTMAP_CELL_H
#define AVT_341_COSTMAP_CELL_H

#include <vector>
#include <limits>
#include <avt_341/node/ros_types.h>
#include <string>
#include "avt_341/avt_341_utils.h"
#include <iostream>
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
      dilated_val = 0;
      terrain = -1.0f;

      // RMS Statistics
      num_points = 0;
      summed_elev = 0.0f;
      avg_elev = 0.0f;
      rms = 0.0f;
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
    uint8_t dilated_val;
    float terrain;

    // RMS Statistics
    int num_points;
    float summed_elev;
    float avg_elev;
    float rms;
    float sum_of_squares;
  };

  class CellObstacleCalculator {
  public:
    virtual ~CellObstacleCalculator() = default;
    virtual bool PastSlopeThreshold(const Cell & cell) const = 0;
    virtual float Slope(const Cell & cell) const = 0;
    virtual void AddOccupancy(const avt_341::msg::PointCloud &point_cloud, std::vector< std::vector<Cell> > & cells, bool dilate) = 0;
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

    static bool IsGridPubMethodValid(const std::string & selected_method){
      const auto valid_methods = {Full, Window, Updates};
      return std::find(valid_methods.begin(), valid_methods.end(), selected_method) != valid_methods.end();
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
  };

  struct DilationSettings
  {
    bool enabled;
    float x;
    float y;
    float proportion;

    int GetNx(const float res) const { return enabled ? lround(x/res) : 0;}
    int GetNy(const float res) const { return enabled ? lround(y/res) : 0;}

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

    utils::vec2 ToPosWorld(const int i, const int j) const {
      return utils::vec2(ToXWorld(i), ToYWorld(j));
    }

    utils::ivec2 ToIdx(const float x, const float y) const {
      return utils::ivec2(ToXIdx(x), ToYIdx(y));
    }

    float ToXWorld(const int i) const { return (i + 0.5f) * res + llx; }
    float ToYWorld(const int j) const { return (j + 0.5f) * res + lly; }
    int ToXIdx(const float x) const { return static_cast<int>((x - llx) / res); }
    int ToYIdx(const float y) const { return static_cast<int>((y - lly) / res); }

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

  };



}

#endif //AVT_341_COSTMAP_CELL_H
