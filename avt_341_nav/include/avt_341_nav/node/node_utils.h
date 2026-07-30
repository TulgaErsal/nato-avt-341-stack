#ifndef AVT_341_NODE_UTILS_H
#define AVT_341_NODE_UTILS_H

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace avt_341_nav {
namespace node {

/// First slash-separated token of the node namespace, e.g. "/veh1/perception"
/// -> "veh1". Empty string for the root namespace.
inline std::string GetLeadingNodeNamespace(const std::string &node_ns) {
  const auto begin = node_ns.find_first_not_of('/');
  if (begin == std::string::npos) {
    return "";
  }
  return node_ns.substr(begin, node_ns.find('/', begin) - begin);
}

inline std::string GetLeadingNodeNamespace(const rclcpp::Node::SharedPtr &node) {
  return GetLeadingNodeNamespace(std::string(node->get_namespace()));
}

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
