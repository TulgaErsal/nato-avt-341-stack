#include "avt_341/mission/mission_manager_dto.h"
#include <sstream>

const std::string MissionMsgType::Formation = "FORM";
const std::string MissionMsgType::Acknowledge = "ACK";
const std::string MissionMsgType::Arrived = "ARRIVE";
const std::string MissionMsgType::TaskComplete = "TASK_COMPLETE";
const std::string MissionMsgType::MoveTo = "MOVETO";
const std::string MissionMsgType::Shutdown = "SHUTDOWN";
const std::string MissionMsgType::SetSpeed = "SET_SPEED";
const std::string MissionMsgType::Cancel = "CANCEL";
const std::string MissionMsgType::CancelAll = "CANCEL_ALL";
const std::string MissionMsgType::Overwatch = "OVERWATCH";
const std::string MissionMsgType::PathFollow = "PATH_FOLLOW";

const std::string PriorityType::QUEUE = "QUEUE";
const std::string PriorityType::QUEUE_SHORT = "Q";
const std::string PriorityType::PREEMPT = "PREEMPT";
const std::string PriorityType::PREEMPT_SHORT = "P";
const std::string PriorityType::CANCEL_ALL_PREVIOUS = "CANCEL_ALL";
const std::string PriorityType::CANCEL_ALL_PREVIOUS_SHORT = "C";

// MissionManagerDto
// =====================================================================================================================
MissionManagerDto::MissionManagerDto(){}

MissionManagerDto::MissionManagerDto(const avt_341::msg::Communication &msg) : MissionManagerDto(msg.sender_name,
                                                                                                      msg.msg_id,
                                                                                                      msg.receiver_name,
                                                                                                      msg.priority_type) {
}

MissionManagerDto::MissionManagerDto(const std::string &sender, int msgId, const std::string &recipient,
                                     const std::string &priority) : sender_name(sender), msg_id(msgId), receiver_name(recipient),
                                                                    priority_type(priority.empty() ? PriorityType::QUEUE : priority) {}

avt_341::msg::Communication MissionManagerDto::toROSMsg() {
  avt_341::msg::Communication msg;
  msg.sender_name = sender_name;
  msg.msg_id = msg_id;
  msg.receiver_name = receiver_name;
  msg.type = getType();
  msg.priority_type = priority_type;
  return msg;
}

// MoveToMsg
// =====================================================================================================================
MoveToMsg::MoveToMsg() : MissionManagerDto() {}

MoveToMsg::MoveToMsg(const avt_341::msg::Communication &msg)
  : MissionManagerDto(msg), objective_name(msg.objective_name), goal_x_offset(msg.x_offset), goal_y_offset(msg.y_offset),
    approach_distance(msg.distance) {
}

MoveToMsg::MoveToMsg(const std::string &sender, int msgId, const std::string &recipient,
                     const std::string &objectiveName, double xOffset, double yOffset, double distance,
                     const std::string &priority)
    : MissionManagerDto(sender, msgId, recipient, priority), objective_name(objectiveName), goal_x_offset(xOffset),
      goal_y_offset(yOffset), approach_distance(distance) {}

avt_341::msg::Communication MoveToMsg::toROSMsg() {
  avt_341::msg::Communication msg = MissionManagerDto::toROSMsg();
  msg.objective_name = objective_name;
  msg.x_offset = goal_x_offset;
  msg.y_scale = goal_y_offset;
  msg.distance = approach_distance;
  return msg;
}

std::string MoveToMsg::getType() { return MissionMsgType::MoveTo; }

// PathFollowMsg
// =====================================================================================================================
PathFollowMsg::PathFollowMsg() : MissionManagerDto() {}

PathFollowMsg::PathFollowMsg(const avt_341::msg::Communication &msg)
  : MissionManagerDto(msg), objective_name(msg.objective_name), desired_speed(msg.desired_speed) {}

PathFollowMsg::PathFollowMsg(const std::string &sender, int msgId, const std::string & recipient,
            const std::string &objectiveName, double desiredSpeed, const std::string &priority)
    : MissionManagerDto(sender, msgId, recipient, priority), objective_name(objectiveName), desired_speed(desiredSpeed) {}

avt_341::msg::Communication PathFollowMsg::toROSMsg() {
  avt_341::msg::Communication msg = MissionManagerDto::toROSMsg();
  msg.objective_name = objective_name;
  msg.desired_speed = desired_speed;
  return msg;
}

std::string PathFollowMsg::getType() { return MissionMsgType::PathFollow; }

// FormationMsg
// =====================================================================================================================
FormationMsg::FormationMsg() : MoveToMsg() {}

FormationMsg::FormationMsg(const avt_341::msg::Communication &msg)
  : MoveToMsg(msg) {
  formation = msg.formation;
  leader_name = msg.leader_name;
  follower1_name = msg.follower1_name;
  follower2_name = msg.follower2_name;
  follower3_name = msg.follower3_name;
  termination_method = msg.termination_method;
  desired_speed = msg.desired_speed;
  x_scale = msg.x_scale > 0.0 ? msg.x_scale : -1.0;
  y_scale = msg.y_scale > 0.0 ? msg.y_scale : -1.0;
}

FormationMsg::FormationMsg(const std::string &sender, int msgId, const std::string & recipient,
  const std::string &objectiveName, const std::string &formation, double desiredSpeed,
  double xOffset, double yOffset, double distance,
  double xScale, double yScale, const std::string & terminationMethod,
  const std::string &priority)
  : MoveToMsg(sender, msgId, recipient, objectiveName, xOffset, yOffset, distance, priority),
  formation(formation), desired_speed(desiredSpeed),
  x_scale(xScale), y_scale(yScale), termination_method(terminationMethod)
{
}

std::string FormationMsg::getType() { return MissionMsgType::Formation; }

avt_341::msg::Communication FormationMsg::toROSMsg(){
  avt_341::msg::Communication msg = MissionManagerDto::toROSMsg();

  msg.formation = formation;
  msg.leader_name = leader_name;
  msg.follower1_name = follower1_name;
  msg.follower2_name = follower2_name;
  msg.follower3_name = follower3_name;
  msg.objective_name = objective_name;
  msg.desired_speed = desired_speed;
  msg.x_scale = x_scale;
  msg.y_scale = y_scale;
  msg.x_offset = goal_x_offset;
  msg.y_offset = goal_y_offset;
  msg.distance = approach_distance;

  return msg;
}

SetSpeedMsg FormationMsg::speedMsg() {
  return SetSpeedMsg{sender_name, msg_id, receiver_name, desired_speed, priority_type};
}


// AcknowledgeMsg
// =====================================================================================================================

AcknowledgeMsg::AcknowledgeMsg(const avt_341::msg::Communication &msg)
: MissionManagerDto(msg), ack_msg_id(msg.target_msg_id) {
}

AcknowledgeMsg::AcknowledgeMsg(const std::string &sender, int msgId, const std::string & recipient, int ackMsdId)
: MissionManagerDto(sender, msgId, recipient), ack_msg_id(ackMsdId){
}

avt_341::msg::Communication AcknowledgeMsg::toROSMsg(){
  avt_341::msg::Communication msg = MissionManagerDto::toROSMsg();
  msg.target_msg_id = ack_msg_id;
  return msg;
}

std::string AcknowledgeMsg::getType() { return MissionMsgType::Acknowledge; }

// ArrivedMsg
// =====================================================================================================================
ArrivedMsg::ArrivedMsg(const avt_341::msg::Communication &msg)
  : MissionManagerDto(msg){

}

ArrivedMsg::ArrivedMsg(const std::string &sender, int msgId, std::string objectiveName)
  : MissionManagerDto(sender, msgId, "", PriorityType::PREEMPT){
  objective_name = objectiveName;
}

avt_341::msg::Communication ArrivedMsg::toROSMsg(){
  avt_341::msg::Communication msg = MissionManagerDto::toROSMsg();
  msg.objective_name = objective_name;
  return msg;
};

std::string ArrivedMsg::getType() { return MissionMsgType::Arrived; }

// ShutdownMsg
// =====================================================================================================================

ShutdownMsg::ShutdownMsg(const avt_341::msg::Communication &msg)
  : MissionManagerDto(msg){
}

ShutdownMsg::ShutdownMsg(const std::string &sender, int msgId, const std::string & receiver,
            const std::string &priority) : MissionManagerDto(sender, msgId, receiver, priority){
}

avt_341::msg::Communication ShutdownMsg::toROSMsg(){
  return MissionManagerDto::toROSMsg();
}
std::string ShutdownMsg::getType() { return MissionMsgType::Shutdown; }

// TaskCompleteMsg
// =====================================================================================================================

TaskCompleteMsg::TaskCompleteMsg(const avt_341::msg::Communication &msg)
    : MissionManagerDto(msg), target_msg_id(msg.target_msg_id){
}

TaskCompleteMsg::TaskCompleteMsg(const std::string &sender, int msgId, const std::string receiver, int completedMsgId)
    : MissionManagerDto(sender, msgId, receiver), target_msg_id(completedMsgId){
}

avt_341::msg::Communication TaskCompleteMsg::toROSMsg(){
  avt_341::msg::Communication msg = MissionManagerDto::toROSMsg();
  msg.target_msg_id = target_msg_id;
  return msg;
}

std::string TaskCompleteMsg::getType() { return MissionMsgType::TaskComplete; }

// SetSpeedMsg
// =====================================================================================================================

SetSpeedMsg::SetSpeedMsg(const avt_341::msg::Communication &msg)
: MissionManagerDto(msg){
  desired_speed = msg.desired_speed;
}

SetSpeedMsg::SetSpeedMsg(const std::string &sender, int msgId, const std::string & receiver,
            double desiredSpeed, const std::string &priority) : MissionManagerDto(sender, msgId, receiver, priority){
  desired_speed = desiredSpeed;
}

avt_341::msg::Communication SetSpeedMsg::toROSMsg() {
  auto msg = MissionManagerDto::toROSMsg();
  msg.desired_speed = desired_speed;
  return msg;
}
std::string SetSpeedMsg::getType() { return MissionMsgType::SetSpeed; }

// CancelMsg
// =====================================================================================================================

CancelMsg::CancelMsg(const avt_341::msg::Communication &msg)
  : MissionManagerDto(msg){
  target_msg_id = msg.target_msg_id;
}

CancelMsg::CancelMsg(const std::string &sender, int msgId, const std::string & recipient,
          int targetMsgId, const std::string &priority)
          : MissionManagerDto(sender, msgId, recipient, priority), target_msg_id(targetMsgId){
}

avt_341::msg::Communication CancelMsg::toROSMsg(){
  auto msg = MissionManagerDto::toROSMsg();
  msg.target_msg_id = target_msg_id;
  return msg;
}
std::string CancelMsg::getType() { return MissionMsgType::Cancel; }

// CancelAllMsg
// =====================================================================================================================

CancelAllMsg::CancelAllMsg(const avt_341::msg::Communication &msg)
    : MissionManagerDto(msg){
}

CancelAllMsg::CancelAllMsg(const std::string &sender, int msgId, const std::string & recipient, const std::string &priority)
    : MissionManagerDto(sender, msgId, recipient, priority){
}

avt_341::msg::Communication CancelAllMsg::toROSMsg(){
  return MissionManagerDto::toROSMsg();
}
std::string CancelAllMsg::getType() { return MissionMsgType::CancelAll; }

// Overwatch
// =====================================================================================================================

OverwatchMsg::OverwatchMsg(const avt_341::msg::Communication &msg)
: MissionManagerDto(msg){
  wait_for_msg_id = msg.target_msg_id;
}

OverwatchMsg::OverwatchMsg(const std::string &sender, int msgId, const std::string & recipient,int waitForMsgId, const std::string &priority)
             : MissionManagerDto(sender, msgId, recipient, priority), wait_for_msg_id(waitForMsgId){
}

avt_341::msg::Communication OverwatchMsg::toROSMsg(){
  avt_341::msg::Communication msg = MissionManagerDto::toROSMsg();
  msg.target_msg_id = wait_for_msg_id;
  return msg;
}
std::string OverwatchMsg::getType() { return MissionMsgType::Overwatch; }

