#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace avt_341 {
namespace testing {

class OccupancyGridNode : public rclcpp::Node {
  public:
    OccupancyGridNode();
    ~OccupancyGridNode() {}
    void TimerCallback();

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        occupancy_grid_pub_;
        cv::Mat image_;
};

} // end namespace Mock
} // end namespace AUTON