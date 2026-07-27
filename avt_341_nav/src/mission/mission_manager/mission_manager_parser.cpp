#include "avt_341/mission/mission_manager_parser.h"
#include <sstream>
#include "avt_341_msgs/msg/communication.hpp"

std::vector<std::string> tokenizeMsg(std::string input){
  std::vector<std::string> tokens;
  size_t pos = 0;
  std::string token;
  while ((pos = input.find(",")) != std::string::npos) {
    token = input.substr(0, pos);
    tokens.push_back(token);
    input.erase(0, pos + 1);
  }
  tokens.push_back(input);
  return tokens;
}

avt_341_msgs::msg::Communication serializedToROSMsg(const std::string & msg) {
  auto tokens = tokenizeMsg(msg);

  avt_341_msgs::msg::Communication message;
  if(tokens.size() < 3){
    return message;
  }
  message.sender_name = tokens[0];
  message.msg_id = atoi(tokens[1].c_str());
  message.type = tokens[2];
  message.priority_type = "Q";
  message.receiver_name = "";
  message.yaw_threshold = -1.0;

  // <sender>,<msg_id>,FORM,<formation>,<leader>,<f1>,<f2>,<f3>,<objective>,<speed>
  // <sender>,<msg_id>,FORM,<formation>,<leader>,<f1>,<f2>,<f3>,<objective>,<speed>,<priority>
  // <sender>,<msg_id>,FORM,<formation>,<leader>,<f1>,<f2>,<f3>,<objective>,<speed>,<x_scale>,<y_scale>,<x_offset>,<y_offset>,<distance>,<yaw_threshold>,<termination_method>,<priority>
  if(message.type == MissionMsgType::Formation) {
    message.formation = tokens[3];
    message.leader_name = tokens[4];
    message.follower1_name = tokens[5];
    message.follower2_name = tokens[6];
    message.follower3_name = tokens[7];
    message.objective_name = tokens[8];
    message.desired_speed = std::stod(tokens[9]);
    message.x_scale = -1.0;
    message.y_scale = -1.0;
    message.x_offset = 0.0;
    message.y_offset = 0.0;
    message.distance = 0.0;

    if(tokens.size() == 11) {
      message.priority_type = tokens[10];
    }else if(tokens.size() > 11) {
      message.x_scale = std::stod(tokens[10]);
      message.y_scale = std::stod(tokens[11]);
      message.x_offset = std::stod(tokens[12]);
      message.y_offset = std::stod(tokens[13]);
      message.distance = std::stod(tokens[14]);
      message.yaw_threshold = std::stod(tokens[15]);
      message.termination_method = tokens[16];
      message.priority_type = tokens[17];
    }
  }
    // <sender>,<msg_id>,ACK,<orig_msg_sender>,<orig_msg_id>
  else if(message.type == MissionMsgType::Acknowledge) {
    message.receiver_name = tokens[3];
    message.target_msg_id = std::stoi(tokens[4]);
  }
    // <sender>,<msg_id>,ARRIVE,<objective>
  else if(message.type == MissionMsgType::Arrived) {
    message.objective_name = tokens[3];
  }
    // <sender>,<msg_id>,TASK_COMPLETE,<orig_msg_sender>,<orig_msg_id>
  else if(message.type == MissionMsgType::TaskComplete) {
    message.receiver_name = tokens[3];
    message.target_msg_id = atoi(tokens[4].c_str());
  }
    // <sender>,<msg_id>,MOVETO,<receiver>,<objective>,<priority>
    // <sender>,<msg_id>,MOVETO,<receiver>,<objective>,<x_offset>,<y_offset>,<distance>,<yaw_threshold>,<priority>
  else if(message.type == MissionMsgType::MoveTo) {
    message.receiver_name = tokens[3];
    message.objective_name = tokens[4];
    message.x_offset = 0.0;
    message.y_offset = 0.0;
    message.distance = 0.0;
    if(tokens.size() == 6) {
      message.priority_type = tokens[5];
    }else if(tokens.size() > 6){
      message.x_offset = std::stod(tokens[5]);
      message.y_offset = std::stod(tokens[6]);
      message.distance = std::stod(tokens[7]);
      message.yaw_threshold = std::stod(tokens[8]);
      message.priority_type = tokens[9];
    }
  }
    // <sender>,<msg_id>,SHUTDOWN,<receiver>
  else if(message.type == MissionMsgType::Shutdown) {
    message.receiver_name = tokens[3];
  }
    // <sender>,<msg_id>,SET_SPEED,<receiver>,<speed>
  else if(message.type == MissionMsgType::SetSpeed) {
    message.receiver_name = tokens[3];
    message.desired_speed = std::stod(tokens[4]);
  }
  else if(message.type == MissionMsgType::Cancel) {
    message.receiver_name = tokens[3];
    message.target_msg_id = atoi(tokens[4].c_str());
  }
  else if(message.type == MissionMsgType::CancelAll) {
    message.receiver_name = tokens[3];
  }
  else if(message.type == MissionMsgType::Overwatch) {
    message.receiver_name = tokens[3];
    message.target_msg_id = atoi(tokens[4].c_str());
  }
  else if(message.type == MissionMsgType::PathFollow) {
    message.receiver_name = tokens[3];
    message.objective_name = tokens[4];
    message.desired_speed = std::stod(tokens[5]);
    message.priority_type = tokens[6];
  }

  return message;
}

std::string rosToSerializedMsg(const avt_341_msgs::msg::Communication & msg){
  std::ostringstream stream;
  stream << msg.sender_name << "," << msg.msg_id << "," << msg.type;

  // <sender>,<msg_id>,FORM,<formation>,<leader>,<f1>,<f2>,<f3>,<objective>,<speed>,<x_scale>,<y_scale>,<x_offset>,<y_offset>,<distance>,<msg.yaw_threshold>,<termination_method>,<priority>
  if(msg.type == MissionMsgType::Formation) {
    stream << "," << msg.formation << "," << msg.leader_name << "," << msg.follower1_name << "," << msg.follower2_name
    << "," << msg.follower3_name << "," << msg.objective_name << "," << msg.desired_speed << "," << msg.x_scale
    << "," << msg.y_scale << "," << msg.x_offset << "," << msg.y_offset << "," << msg.distance << "," << msg.yaw_threshold
    << "," << msg.termination_method << "," << msg.priority_type;
  }
  // <sender>,<msg_id>,ACK,<orig_msg_sender>,<orig_msg_id>
  else if(msg.type == MissionMsgType::Acknowledge) {
    stream << "," << msg.receiver_name << "," << msg.target_msg_id;
  }
  // <sender>,<msg_id>,ARRIVE,<objective>
  else if(msg.type == MissionMsgType::Arrived) {
    stream << "," << msg.objective_name;
  }
  // <sender>,<msg_id>,TASK_COMPLETE,<orig_msg_sender>,<orig_msg_id>
  else if(msg.type == MissionMsgType::TaskComplete) {
    stream << "," << msg.receiver_name << "," << msg.target_msg_id;
  }
  // <sender>,<msg_id>,MOVETO,<receiver>,<objective>,<x_offset>,<y_offset>,<distance>,<yaw_threshold>,<priority>
  else if(msg.type == MissionMsgType::MoveTo) {
    stream << "," << msg.receiver_name << "," << msg.objective_name << "," << msg.x_offset << "," << msg.y_offset
    << "," << msg.distance << "," << msg.yaw_threshold << "," << msg.priority_type;
  }
  // <sender>,<msg_id>,SHUTDOWN,<receiver>,<priority>
  else if(msg.type == MissionMsgType::Shutdown) {
    stream << "," << msg.receiver_name;
  }
  // <sender>,<msg_id>,SET_SPEED,<receiver>,<speed>,<priority>
  else if(msg.type == MissionMsgType::SetSpeed) {
    stream << "," << msg.receiver_name << "," << msg.desired_speed;
  }
  // <sender>,<msg_id>,CANCEL,<receiver>,<target_msg_id>,<priority>
  else if(msg.type == MissionMsgType::Cancel) {
    stream << "," << msg.receiver_name << "," << msg.target_msg_id;
  }
  // <sender>,<msg_id>,CANCEL_ALL,<receiver>,<priority>
  else if(msg.type == MissionMsgType::CancelAll) {
    stream << "," << msg.receiver_name << "," << msg.priority_type;
  }
  // <sender>,<msg_id>,OVERWATCH,<receiver>,<wait_for_task_id>,<priority>
  else if(msg.type == MissionMsgType::Overwatch) {
    stream << "," << msg.receiver_name << "," << msg.target_msg_id << "," << msg.priority_type;
  }
  // <sender>,<msg_id>,PATH_FOLLOW,<receiver>,<path_id>,<speed>,<priority>
  else if(msg.type == MissionMsgType::PathFollow) {
    stream << "," << msg.receiver_name << "," << msg.objective_name << "," << msg.desired_speed << "," << msg.priority_type;
  }

  return stream.str();

}

avt_341_msgs::msg::Communication concreteToROSMsg(MissionManagerDto * msg){
  return msg->toROSMsg();
}

std::shared_ptr<MissionManagerDto> rosToConcreteMsg(const avt_341_msgs::msg::Communication & msg){
  if(msg.type == MissionMsgType::Formation){
    return std::make_shared<FormationMsg>(msg);
  }
  if(msg.type == MissionMsgType::Acknowledge){
    return std::make_shared<AcknowledgeMsg>(msg);
  }
  if(msg.type == MissionMsgType::Arrived){
    return std::make_shared<ArrivedMsg>(msg);
  }
  if(msg.type == MissionMsgType::TaskComplete){
    return std::make_shared<TaskCompleteMsg>(msg);
  }
  if(msg.type == MissionMsgType::MoveTo){
    return std::make_shared<MoveToMsg>(msg);
  }
  if(msg.type == MissionMsgType::Shutdown){
    return std::make_shared<ShutdownMsg>(msg);
  }
  if(msg.type == MissionMsgType::SetSpeed){
    return std::make_shared<SetSpeedMsg>(msg);
  }
  if(msg.type == MissionMsgType::Cancel){
    return std::make_shared<CancelMsg>(msg);
  }
  if(msg.type == MissionMsgType::CancelAll){
    return std::make_shared<CancelAllMsg>(msg);
  }
  if(msg.type == MissionMsgType::Overwatch){
    return std::make_shared<OverwatchMsg>(msg);
  }
  if(msg.type == MissionMsgType::PathFollow){
    return std::make_shared<PathFollowMsg>(msg);
  }
  return nullptr;
}

std::shared_ptr<MissionManagerDto> serializedToConcreteMsg(const std::string & msg){
  auto ros_msg = serializedToROSMsg(msg);
  return rosToConcreteMsg(ros_msg);
}

std::string concreteToSerializedMsg(MissionManagerDto * msg){
  auto ros_msg = msg->toROSMsg();
  return rosToSerializedMsg(ros_msg);
}

bool isMsgFor(const std::string & veh, const avt_341_msgs::msg::Communication & msg){
  return msg.type == MissionMsgType::TaskComplete
         || msg.type == MissionMsgType::Arrived
         || (msg.type == MissionMsgType::Formation && isVehicleInFormation(veh, msg))
         || msg.receiver_name == veh;
}

bool isMsgFor(const std::string & veh, MissionManagerDto* msg){
  return isMsgFor(veh, msg->toROSMsg());
}

bool isVehicleInFormation(const std::string & veh, const avt_341_msgs::msg::Communication & msg){
  return msg.leader_name == veh || msg.follower1_name == veh
         || msg.follower2_name == veh || msg.follower3_name == veh;
}
