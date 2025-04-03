#ifndef AVT_341_ELEVATION_GRID_CELL_H
#define AVT_341_ELEVATION_GRID_CELL_H

#include <vector>
#include <limits>
#include <string>
#include <iostream>

namespace avt_341{
namespace perception{

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
    //float low = std::numeric_limits<float>::max();
    //float high = std::numeric_limits<float>::lowest();
    //float highest = std::numeric_limits<float>::lowest();
    //float second_highest = std::numeric_limits<float>::lowest();
    constexpr static const float MIN_LIMIT = std::numeric_limits<float>::lowest();
    constexpr static const float MAX_LIMIT = std::numeric_limits<float>::max();
  public:
    Cell(){
      low.val = MAX_LIMIT;
      high.val = MIN_LIMIT;
      highest.val = MIN_LIMIT;
      second_highest.val = MIN_LIMIT;
      has_dilated = false;
      dilated_val = 0;
      terrain = 0.0f;
      dilated_age = 0.0f;
    }

    void AgeCell(float dt){
      low.age += dt;
      high.age += dt;
      highest.age += dt;
      second_highest.age += dt;
      dilated_age += dt;
    }

    void ResetHeight(){
      // Need to keep dilated_val since from other adjacent cell
      low.val = MAX_LIMIT;
      high.val = MIN_LIMIT;
    }


    ElevAge low,high,highest,second_highest;

    inline float height() const { return high.val - low.val; }
    inline bool filled() const { return low.val < MAX_LIMIT; }

    bool has_dilated; //  = false;
    uint8_t dilated_val; //  = 0;
    float dilated_age;
    float terrain; //  = 0.0f;
  };

  class CellObstacleCalculator {
  public:
    virtual ~CellObstacleCalculator() = default;
    virtual bool PastSlopeThreshold(const Cell & cell) const = 0;
    virtual float Slope(const Cell & cell) const = 0;
    virtual void AddOccupancy(const avt_341::msg::PointCloud &point_cloud, std::vector< std::vector<Cell> > & cells, bool dilate) = 0;
  };

} // namespace perception
} // namespace avt_341

#endif //AVT_341_ELEVATION_GRID_CELL_H