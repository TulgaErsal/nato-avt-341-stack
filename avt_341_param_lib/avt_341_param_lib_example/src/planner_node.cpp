#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <avt_341_param_lib_example/nav_parameters.hpp>

namespace nav_demo {

class PlannerNode : public rclcpp::Node {
 public:
  explicit PlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("planner", options) {
    param_listener_ = std::make_shared<ParamsListener>(get_node_parameters_interface(), get_logger());
    params_ = param_listener_->get_params();
    RCLCPP_INFO(get_logger(), "cruise_speed: '%f'", params_.cruise_speed);
    RCLCPP_INFO(get_logger(), "planner_mode: '%s'", params_.planner_mode.c_str());
  }

 private:
  std::shared_ptr<ParamsListener> param_listener_;
  Params params_;
};

}  // namespace nav_demo

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav_demo::PlannerNode>());
  return EXIT_SUCCESS;
}
