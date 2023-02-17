#ifndef AVT_341_ELEVATION_GRID_CELL_H
#define AVT_341_ELEVATION_GRID_CELL_H

#include <vector>
#include <limits>
#include <string>

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
  public:
    Cell(){

      low.val = std::numeric_limits<float>::max();
      high.val = std::numeric_limits<float>::lowest();
      highest.val = std::numeric_limits<float>::lowest();
      second_highest.val = std::numeric_limits<float>::lowest();
      height = 0.0f;
      filled = false;
      slope = 0.0f;
      obstacle = false;
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
    ElevAge low,high,highest,second_highest;
    //low.val = std::numeric_limits<float>::max();
    //high.val = std::numeric_limits<float>::lowest();
    //highest.val = std::numeric_limits<float>::lowest();
    //second_highest.val = std::numeric_limits<float>::lowest();
    float height; // = 0.0f;
    bool filled; //  = false;
    //float slope_x = 0.0f;
    //float slope_y = 0.0f;
    float slope; //  = 0.0f;
    bool obstacle; //  = false;
    bool has_dilated; //  = false;
    uint8_t dilated_val; //  = 0;
    float dilated_age;
    float terrain; //  = 0.0f;
  };

} // namespace perception
} // namespace avt_341

#endif //AVT_341_ELEVATION_GRID_CELL_H