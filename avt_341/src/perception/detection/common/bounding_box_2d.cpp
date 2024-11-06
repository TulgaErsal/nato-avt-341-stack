#include <avt_341/perception/detection/common/bounding_box_2d.hpp>

namespace avt_341 {
namespace perception {

BoundingBox2D::BoundingBox2D() {}

BoundingBox2D::BoundingBox2D(const int x_min,
                             const int x_max,
                             const int y_min,
                             const int y_max)
    : x_min_(x_min),
      x_max_(x_max),
      y_min_(y_min),
      y_max_(y_max) {}

sensor_msgs::msg::RegionOfInterest
BoundingBox2D::ToROSRegionOfInterestMessage() {
    sensor_msgs::msg::RegionOfInterest message;
    message.x_offset = x_min_;
    message.y_offset = y_min_;
    message.height = GetHeight();
    message.width = GetWidth();
    message.do_rectify = false;
    return message;
}

vision_msgs::msg::BoundingBox2D BoundingBox2D::ToROSVisionMessage() {
    vision_msgs::msg::BoundingBox2D message;
    message.center.position.x = GetCenterX();
    message.center.position.y = GetCenterY();
    // Bounding boxes from YOLO object detectors do not allow for rotation,
    // hence the bounding box angle is hardcoded to null.
    message.center.theta = 0.0;
    message.size_x = GetWidth();
    message.size_y = GetHeight();
    return message;
}

int BoundingBox2D::GetCenterX() { return std::floor((x_min_ + x_max_) / 2.0); }

int BoundingBox2D::GetCenterY() { return std::floor((y_min_ + y_max_) / 2.0); }

int BoundingBox2D::GetHeight() { return y_max_ - y_min_; }

int BoundingBox2D::GetWidth() { return x_max_ - x_min_; }

int BoundingBox2D::GetXMax() { return x_max_; }

int BoundingBox2D::GetXMin() { return x_min_; }

int BoundingBox2D::GetYMax() { return y_max_; }

int BoundingBox2D::GetYMin() { return y_min_; }

} // namespace perception
} // namespace avt_341