#include <avt_341_param_lib/parameter_validators.hpp>

namespace avt_341_param_lib {

auto to_parameter_result_msg(ValidationResult const& result)
    -> rcl_interfaces::msg::SetParametersResult {
      auto msg = rcl_interfaces::msg::SetParametersResult();
      msg.successful = !result.has_value();
      msg.reason = result.has_value() ? result.value() : "";
      return msg;
}

}
