#include <memory>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>

#include <avt_341_param_lib_example/mixin_ex_params_service.hpp>

namespace mixin_ex {

// The inherited mixin is a real base class, not just a copy of its members.
static_assert(
    std::is_base_of_v<example_demo::mixins::InheritedParams, Params>);

class MixinExampleNode : public rclcpp::Node {
 public:
  explicit MixinExampleNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("mixin_ex", options) {
    param_listener_ = std::make_shared<ParamsListener>(get_node_parameters_interface(), get_logger());
    params_ = param_listener_->get_params();

    // Inherited from inherited_mixin: a base-class member, so no name prefix.
    RCLCPP_INFO(get_logger(), "rate_hz: '%f'", params_.rate_hz);

    // Composed from composed_mixin: a member nested at its mount point.
    RCLCPP_INFO(get_logger(), "region.extents.width: '%f'", params_.region.extents.width);
    RCLCPP_INFO(get_logger(), "region.label: '%s'", params_.region.label.c_str());

    // Because the mixin is a base class, the whole parameter set can be passed
    // to code that only knows about the mixin's own type.
    log_inherited(params_);
  }

 private:
  void log_inherited(const example_demo::mixins::InheritedParams& inherited) const {
    RCLCPP_INFO(get_logger(), "warmup_time: '%f'", inherited.warmup_time);
  }

  std::shared_ptr<ParamsListener> param_listener_;
  Params params_;
};

}  // namespace mixin_ex

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mixin_ex::MixinExampleNode>());
  return EXIT_SUCCESS;
}
