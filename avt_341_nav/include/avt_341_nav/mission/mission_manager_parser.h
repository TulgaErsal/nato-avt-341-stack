#ifndef AVT_341_MISSION_MANAGER_PARSER_H
#define AVT_341_MISSION_MANAGER_PARSER_H

#include "avt_341_msgs/msg/communication.hpp"
#include "avt_341_nav/mission/mission_manager_dto.h"

std::vector<std::string> tokenizeMsg(std::string input);

bool isMsgFor(const std::string & veh, const avt_341_msgs::msg::Communication & msg);
bool isMsgFor(const std::string & veh, MissionManagerDto* msg);
bool isVehicleInFormation(const std::string & veh, const avt_341_msgs::msg::Communication & msg);

avt_341_msgs::msg::Communication serializedToROSMsg(const std::string & msg);
avt_341_msgs::msg::Communication concreteToROSMsg(MissionManagerDto * msg);

std::shared_ptr<MissionManagerDto> rosToConcreteMsg(const avt_341_msgs::msg::Communication & msg);
std::shared_ptr<MissionManagerDto> serializedToConcreteMsg(const std::string & msg);

std::string rosToSerializedMsg(const avt_341_msgs::msg::Communication & msg);
std::string concreteToSerializedMsg(MissionManagerDto * msg);

#endif //AVT_341_MISSION_MANAGER_PARSER_H
