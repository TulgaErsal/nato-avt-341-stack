#ifndef AVT_341_MISSION_MANAGER_SERIALIZATION_H
#define AVT_341_MISSION_MANAGER_SERIALIZATION_H

#include "avt_341/node/ros_types.h"
#include "avt_341/mission/mission_manager_dto.h"

std::vector<std::string> tokenizeMsg(std::string input);
avt_341::msg::Communication serializedToROSMsg(const std::string & msg);
std::string rosToSerializedMsg(const avt_341::msg::Communication & msg);

#endif //AVT_341_MISSION_MANAGER_SERIALIZATION_H
