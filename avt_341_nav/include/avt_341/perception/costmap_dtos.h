#ifndef AVT_341_COSTMAP_CELL_H
#define AVT_341_COSTMAP_CELL_H

#include <vector>
#include <limits>
#include "sensor_msgs/msg/point_cloud.hpp"
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
    virtual void AddOccupancy(const sensor_msgs::msg::PointCloud &point_cloud, std::vector< std::vector<Cell> > & cells, bool dilate) = 0;
    virtual bool PastSlopeThreshold(const Cell & cell) const = 0;
    virtual float Slope(const Cell & cell) const = 0;
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

  struct PolygonZone
  {
    std::string label;
    std::vector<utils::vec2> vertices;
    double occ_value;
    int seg_value;
  };


}

#endif //AVT_341_COSTMAP_CELL_H
