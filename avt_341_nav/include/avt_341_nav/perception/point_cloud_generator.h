#ifndef AVT_341_POINT_CLOUD_GENERATOR_H
#define AVT_341_POINT_CLOUD_GENERATOR_H

#include "avt_341_nav/core/math_dto.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace avt_341_nav {
  namespace perception {
    class PointCloudGenerator {
      public:
        static void toROSMsg(const std::vector<avt_341_nav::core::vec3> & points, sensor_msgs::msg::PointCloud2 & out_point_cloud);
        static void toROSMsg(const std::vector<avt_341_nav::core::vec3> & points, const std::vector<int> & seg_values, sensor_msgs::msg::PointCloud2 & out_point_cloud);
    };
  }
}

#endif //AVT_341_POINT_CLOUD_GENERATOR_H
