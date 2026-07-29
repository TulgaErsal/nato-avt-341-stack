#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <avt_341_param_lib_example/sensor_params_service.hpp>

namespace sensor_demo {

class SensorNode : public rclcpp::Node {
 public:
  explicit SensorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("sensor", options) {
    param_listener_ = std::make_shared<ParamsListener>(get_node_parameters_interface(), get_logger());
    params_ = param_listener_->get_params();
    RCLCPP_INFO(get_logger(), "rate: '%f'", params_.rate);
    RCLCPP_INFO(get_logger(), "enabled: '%s'", params_.enabled ? "true" : "false");
  }

 private:
  std::shared_ptr<ParamsListener> param_listener_;
  Params params_;
};

}  // namespace sensor_demo

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensor_demo::SensorNode>());
  return EXIT_SUCCESS;
}
