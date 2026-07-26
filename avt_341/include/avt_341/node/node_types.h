#ifndef AVT_341_NODE_TYPES_H
#define AVT_341_NODE_TYPES_H

#include <string>

namespace avt_341 {
namespace node {

class NodeType {
public:
  inline static const std::string LocalPlanner = "local_planner";
  inline static const std::string GlobalPlanner = "global_planner";
  inline static const std::string Control = "control";
  inline static const std::string Perception = "perception";
  inline static const std::string Mission = "mission";
};

} // namespace node
} // namespace avt_341

#endif // AVT_341_NODE_TYPES_H
