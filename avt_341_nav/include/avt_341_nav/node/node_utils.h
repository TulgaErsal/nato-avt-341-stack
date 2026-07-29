#ifndef AVT_341_NODE_UTILS_H
#define AVT_341_NODE_UTILS_H

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace avt_341_nav {
namespace node {

// Declares the parameter if not yet declared, then reads it. Accepts legacy
// "~"-prefixed private parameter names.
template <typename ParameterT>
void get_parameter(const rclcpp::Node::SharedPtr &node, const std::string &name,
                   ParameterT &parameter_out, const ParameterT &default_value) {
  const std::string name_local = (!name.empty() && name[0] == '~') ? name.substr(1) : name;
  if (!node->has_parameter(name_local)) {
    node->declare_parameter(name_local, default_value);
  }
  node->get_parameter(name_local, parameter_out);
}

} // namespace node
} // namespace avt_341_nav

#endif // AVT_341_NODE_UTILS_H
