#ifndef AVT_341_MISSION_MANAGER_DTO_H
#define AVT_341_MISSION_MANAGER_DTO_H

#include "avt_341/node/ros_types.h"

class PriorityType{
public:
  static const std::string QUEUE;
  static const std::string QUEUE_SHORT;
  static const std::string PREEMPT;
  static const std::string PREEMPT_SHORT;
  static const std::string CANCEL_ALL_PREVIOUS;
  static const std::string CANCEL_ALL_PREVIOUS_SHORT;
  inline static bool isQueue(const std::string &type) { return type.empty() || type == QUEUE || type == QUEUE_SHORT; }
  inline static bool isPreempt(const std::string &type) { return type == PREEMPT || type == PREEMPT_SHORT; }
  inline static bool isCancelAllPrevious(const std::string &type) { return type == CANCEL_ALL_PREVIOUS || type == CANCEL_ALL_PREVIOUS_SHORT; }
};

struct MissionMsgType {
  static const std::string Formation;
  static const std::string Acknowledge;
  static const std::string Arrived;
  static const std::string TaskComplete;
  static const std::string MoveTo;
  static const std::string Shutdown;
  static const std::string SetSpeed;
  static const std::string Cancel;
  static const std::string CancelAll;
  static const std::string Overwatch;
};

struct MissionManagerDto {
  MissionManagerDto();
  explicit MissionManagerDto(const avt_341::msg::Communication &msg);
  MissionManagerDto(const std::string &sender, int msgId, const std::string & recipient,
                    const std::string &priority = PriorityType::QUEUE);

  virtual avt_341::msg::Communication toROSMsg();
  virtual std::string getType() = 0;

  std::string sender_name;
  int msg_id;
  std::string receiver_name;
  std::string priority_type;

};

struct MoveToMsg : public MissionManagerDto {
  MoveToMsg();
  explicit MoveToMsg(const avt_341::msg::Communication &msg);
  MoveToMsg(const std::string &sender, int msgId, const std::string & recipient,
            const std::string &objectiveName, double xOffset = 0.0, double yOffset = 0.0, double distance = 0.0,
            const std::string &priority = PriorityType::QUEUE);

  avt_341::msg::Communication toROSMsg() override;
  std::string getType() override;

  std::string objective_name;
  double goal_x_offset;
  double goal_y_offset;
  double approach_distance;
};

struct SetSpeedMsg : public MissionManagerDto {
  explicit SetSpeedMsg(const avt_341::msg::Communication &msg);
  SetSpeedMsg(const std::string &sender, int msgId, const std::string & receiver,
              double desiredSpeed, const std::string &priority = PriorityType::QUEUE);
  avt_341::msg::Communication toROSMsg() override;
  std::string getType() override;

  double desired_speed;
};

struct FormationMsg : public MoveToMsg {
  FormationMsg();
  explicit FormationMsg(const avt_341::msg::Communication &msg);
  FormationMsg(const std::string &sender, int msgId, const std::string & recipient,
               const std::string &objectiveName, const std::string &formation, double desiredSpeed,
               double xOffset=0.0, double yOffset=0.0, double distance=0.0,
               double xScale=-1.0, double yScale=-1.0, const std::string & terminationMethod = "",
               const std::string &priority = PriorityType::QUEUE);

  avt_341::msg::Communication toROSMsg() override;
  std::string getType() override;
  SetSpeedMsg speedMsg();

  std::string formation;
  std::string leader_name;
  std::string follower1_name;
  std::string follower2_name;
  std::string follower3_name;
  std::string termination_method;
  double x_scale;
  double y_scale;
  double desired_speed;
};

struct AcknowledgeMsg : public MissionManagerDto {
  explicit AcknowledgeMsg(const avt_341::msg::Communication &msg);

  AcknowledgeMsg(const std::string &sender, int msgId, const std::string & recipient,
                 int ackMsdId);
  avt_341::msg::Communication toROSMsg() override;
  std::string getType() override;

  int ack_msg_id;
};

struct ArrivedMsg : public MissionManagerDto {
  explicit ArrivedMsg(const avt_341::msg::Communication &msg);
  ArrivedMsg(const std::string &sender, int msgId, std::string objectiveName);

  avt_341::msg::Communication toROSMsg() override;
  std::string getType() override;
  std::string objective_name;
};

struct ShutdownMsg : public MissionManagerDto {
  explicit ShutdownMsg(const avt_341::msg::Communication &msg);
  ShutdownMsg(const std::string &sender, int msgId, const std::string & receiver,
              const std::string &priority = PriorityType::QUEUE);

  avt_341::msg::Communication toROSMsg() override;
  std::string getType() override;
};

struct TaskCompleteMsg : public MissionManagerDto {
  explicit TaskCompleteMsg(const avt_341::msg::Communication &msg);
  TaskCompleteMsg(const std::string &sender, int msgId, const std::string receiver, int completedMsgId);
  avt_341::msg::Communication toROSMsg() override;
  std::string getType() override;

  int completed_msg_id;
};

struct CancelMsg : public MissionManagerDto {
  explicit CancelMsg(const avt_341::msg::Communication &msg);
  CancelMsg(const std::string &sender, int msgId, const std::string & recipient,
            int targetMsgId, const std::string &priority = PriorityType::PREEMPT);
  avt_341::msg::Communication toROSMsg() override;
  std::string getType() override;

  int target_msg_id;
};

struct CancelAllMsg : public MissionManagerDto {
  explicit CancelAllMsg(const avt_341::msg::Communication &msg);
  CancelAllMsg(const std::string &sender, int msgId, const std::string & recipient,
               const std::string &priority = PriorityType::PREEMPT);
  avt_341::msg::Communication toROSMsg() override;
  std::string getType() override;

};

struct OverwatchMsg : public MissionManagerDto {
  explicit OverwatchMsg(const avt_341::msg::Communication &msg);
  OverwatchMsg(const std::string &sender, int msgId, const std::string & recipient,int waitForMsgId,
               const std::string &priority = PriorityType::QUEUE);
  avt_341::msg::Communication toROSMsg() override;
  std::string getType() override;

  int wait_for_msg_id;
};

#endif
