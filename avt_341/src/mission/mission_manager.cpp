// clas definition
#include "avt_341/mission/mission_manager.h"
#include <fstream>
#include <sstream>

namespace avt_341 {
namespace mission {

const std::string PriorityType::QUEUE = "QUEUE";
const std::string PriorityType::QUEUE_SHORT = "Q";
const std::string PriorityType::PREEMPT = "PREEMPT";
const std::string PriorityType::PREEMPT_SHORT = "P";
const std::string PriorityType::CANCEL_ALL_PREVIOUS = "CANCEL_ALL";
const std::string PriorityType::CANCEL_ALL_PREVIOUS_SHORT = "C";

MissionManager::MissionManager(const FormationParameters & formation_params, const ToiParameters & toi_params, std::shared_ptr<node::NodeProxy> node_proxy, bool add_name_id_to_msg)
: formation_params(formation_params), toi_params_(toi_params), node_proxy_(node_proxy), add_name_id_to_msg_(add_name_id_to_msg){

    my_name = formation_params.my_name;
    nav_state = avt_341::utils::NavStackState::NotInit;

    arrival_announced = true;

    mission_data.clear();
    task_list.clear();
    mission_contacts.clear();

    waypoint_pub = node_proxy_->create_publisher<avt_341::msg::Path>("avt_341/new_waypoints", 10);
    reset_pub = node_proxy_->create_publisher<avt_341::msg::String>("avt_341/reset", 10);
    gp_path_pub = node_proxy_->create_publisher<avt_341::msg::Path>("avt_341/global_path", 10);
    navcommand_pub = node_proxy_->create_publisher<avt_341::msg::Int32>("avt_341/nav_command_state", 10);
    communication_pub = node_proxy_->create_publisher<avt_341::msg::String>("avt_341/comm_messages", 100);
    gp_toggle_pub = node_proxy_->create_publisher<avt_341::msg::Int32>("avt_341/gp_toggle", 10);
    speed_pub = node_proxy_->create_publisher<avt_341::msg::Float64>("avt_341/desired_speed", 10);
}

MissionManager::~MissionManager() {
    // Clean up the memory allocated to tasks
    reset();
}

std::vector<std::string> getLine(std::istream& stream) {
    std::vector<std::string> result;
    std::string line;
    std::getline(stream, line);
    std::stringstream lineStream(line);
    std::string cell;

    while(std::getline(lineStream, cell, ',')) {
        result.push_back(cell);
    }
    return result;
}

int MissionManager::loadMissionDefinition(std::string filename) {
    std::string line;
    std::vector<std::string> contents;
    MissionPoint missionPt;

    // Load the mission from file
    std::ifstream infile(filename);
    if(infile.is_open()) {
        mission_data.clear();
        while(std::getline(infile, line))
        {
            std::istringstream iss(line);
            //std::cout << line << std::endl;
            contents = getLine(iss);
            if(!contents.empty() && strcmp(contents[0].c_str(),"name")) {
                missionPt.name = contents[0];
                missionPt.pos_x = std::strtod(contents[1].c_str(), NULL);
                missionPt.pos_y = std::strtod(contents[2].c_str(), NULL);
                missionPt.pos_z = std::strtod(contents[3].c_str(), NULL);
                missionPt.rot_x = std::strtod(contents[4].c_str(), NULL);
                missionPt.rot_y = std::strtod(contents[5].c_str(), NULL);
                missionPt.rot_z = std::strtod(contents[6].c_str(), NULL);
                missionPt.rot_w = std::strtod(contents[7].c_str(), NULL);
                mission_data.push_back(missionPt);
                //std::cout << "Pose: " << position.name << " " << position.pos_x << " " << position.rot_w << std::endl;
            }
        }    
    } else {
        node_proxy_->log_info("Error reading mission definition %s", filename.c_str());
    }

    // Find overwatch positions, assume starting with SP_
    overwatch_positions.clear();
    for(const auto & mp : mission_data){
      if(mp.name.rfind("SP_", 0) == 0){
        overwatch_positions.push_back(mp);
      }
    }

    return 0;
}

bool MissionManager::getMissionPoint(MissionPoint& mission_point, std::string name) {
    auto it = std::find_if(std::begin(mission_data), std::end(mission_data), 
                [&](const auto& e) {return e.name == name; });
    if(it == mission_data.end()) {
        node_proxy_->log_info("Missing Mission Point %s", name.c_str());
        return false;
    }
//    node_proxy_->log_info("Found Mission Point %s = (%.2f, %.2f, %.2f) found", name.c_str(), it->pos_x, it->pos_y, it->pos_z);
    mission_point = *it;
    return true;
}


bool MissionManager::addTask(Task* task, const std::string & priority_type) {
    // if no active task, put it on the list

    if(PriorityType::isCancelAllPrevious(priority_type)){
      resetTaskList(true);
    }

    bool is_preempt = PriorityType::isPreempt(priority_type);
    if(is_preempt) {
      Task* preempted_task = currentTask();
      if(preempted_task == nullptr || preempted_task->is_preemptable){
        task_list.push_front(task);
      }else{
        // Insert at front before first preemptable task
        auto it = std::find_if(task_list.begin(), task_list.end(), [&](Task* t){return t->is_preemptable;});
        preempted_task = *it;
        task_list.insert(it, task);
      }

      if(preempted_task != nullptr){
        preempted_task->onPreempt();
      }
      node_proxy_->log_info("%s PREEMPT %s", my_name.c_str(), task->description().c_str());
    }else{
      task_list.push_back(task);
      node_proxy_->log_info("%s QUEUED %s", my_name.c_str(), task->description().c_str());
    }
    return true;
}

void MissionManager::publishGoal(const avt_341::msg::PoseStamped & target_pose){
    avt_341::msg::Path goal_msg;
    goal_msg.poses.clear();
    goal_msg.poses.push_back(target_pose);
    goal_msg.header.stamp = node_proxy_->get_stamp();
    goal_msg.header.frame_id = "map";
    waypoint_pub->publish(goal_msg);
}

void MissionManager::publishPath(const avt_341::msg::Path& path){
  avt_341::msg::Path path_msg;
  path_msg.header.stamp = node_proxy_->get_stamp();
  path_msg.header.frame_id = "map";
  path_msg.poses = path.poses;
  gp_path_pub->publish(path);
}

void MissionManager::publishGpToggle(int state){
  avt_341::msg::Int32 nav_msg;
  nav_msg.data = state;
  gp_toggle_pub->publish(nav_msg);
}

void MissionManager::publishNavStateCmd(int state){
    avt_341::msg::Int32 nav_msg;
    nav_msg.data = state;
    navcommand_pub->publish(nav_msg);
}

void MissionManager::publishCommStr(const std::string & msg_data){
    avt_341::msg::String msg;
    msg.data = msg_data;
    communication_pub->publish(msg);
}

void MissionManager::publishTaskCompletion(Task * task){
  publishTaskCompletion(task->sender_name, task->msg_id);
}

//void MissionManager::publishFormationStatus(avt_341::msg::FollowerStatus & status_msg){
//  follower_status_pub->publish(status_msg);
//}

void MissionManager::publishTaskCompletion(const std::string & sender_name, int msg_id){
  std::ostringstream stream;
  if(add_name_id_to_msg_){
    stream << sender_name << "," << msg_id << "," << "TASK_COMPLETE," << sender_name << "," << msg_id;
  }else{
    stream << "TASK_COMPLETE," << sender_name << "," << msg_id;
  }
  avt_341::msg::String comm_msg;
  comm_msg.data = stream.str();
  communication_pub->publish(comm_msg);
}

void MissionManager::updateTasks() {
    Task* active_task = currentTask();
    if(active_task != nullptr) {

        if(!active_task->init_done){
          active_task->init();
          node_proxy_->log_info("    > %s EXECUTING (of %d) %s", my_name.c_str(), task_list.size(), active_task->description().c_str());
        }

        active_task->run();
        if(active_task->is_done()){
          active_task->on_done();
          node_proxy_->log_info("    > %s TASK COMPLETE (of %d): %s", my_name.c_str(), task_list.size(), active_task->description().c_str());
          publishTaskCompletion(active_task);
          task_list.pop_front();
          delete active_task;
        }
    }
}

Task* MissionManager::currentTask(){
  return task_list.empty() ? nullptr : task_list.front();
}

void MissionManager::postUpdateTasks() {
}

// Contact Management
bool MissionManager::hasContact(const std::string & name, const avt_341::msg::PoseStamped & pose) {
	std::vector<Contact>::iterator it = std::find_if(std::begin(mission_contacts), std::end(mission_contacts),
	            [&](const Contact& e) {return (e.name == name && IsClose(e.pose.pose, pose.pose, sodist_threshold)); });
  return it != mission_contacts.end();
}

auto MissionManager::getClosestNewContact() {
    auto it = std::min_element(mission_contacts.begin(), mission_contacts.end(), [&](const Contact& a, const Contact& b) {
        if((a.investigating || a.investigated) && (b.investigating || b.investigated)) {
            return false;
        }
        if(a.investigating || a.investigated) {
            return false; // Contact b is closer
        }
        if(b.investigating || b.investigated) {
            return true; // Contact a is closer
        }
        double da = PosePlanarDistanceSq(odometry.pose.pose.position, a.pose.pose.position);
        double db = PosePlanarDistanceSq(odometry.pose.pose.position, b.pose.pose.position);
        return da < db;
    });
    return it;
}

void MissionManager::addContact(const std::string & name, const avt_341::msg::PoseStamped & pose) {
  Contact new_contact;
  new_contact.name = name;
  new_contact.pose = pose;
  new_contact.investigating = false;
  new_contact.investigated = false;
  new_contact.is_new = true;
  mission_contacts.push_back(new_contact);
}

void MissionManager::resetTaskList(bool send_completion_msg) {
  node_proxy_->log_info("%s CANCEL_ALL: Clearing task list of size %d.",  my_name.c_str(), task_list.size());
  for(auto task : task_list) {
    cancelTask(task->msg_id, send_completion_msg);
  }
  task_list.clear();
}

void MissionManager::reset(){
  resetTaskList(false);
  obj_detection_cnt=9999; // TODO: Hack for task ids of contacts, replace later
  task_completions_.clear();
  current_gp_goal = avt_341::msg::PoseStamped();
  mission_contacts.clear();

  avt_341::msg::String reset_msg;
  reset_msg.data = avt_341::node::NodeType::GlobalPlanner;
  reset_pub->publish(reset_msg);
}

void MissionManager::cancelTask(int task_id, bool send_completion_msg){
  auto it = std::find_if(std::begin(task_list), std::end(task_list),
                         [task_id](const auto& e) {return e->msg_id == task_id; });
  if(it != task_list.end()){
    Task* task = *it;
    task->onPreempt();
    node_proxy_->log_info("%s CANCEL TASK: %s", my_name.c_str(), task->description().c_str());
    if(send_completion_msg){
      publishTaskCompletion(task);
    }
    task_list.erase(it);
    delete task;
  }
}

// Message Handlers
void MissionManager::handleContacts(const avt_341::msg::Path & contacts, const std::map<std::string, avt_341::msg::Odometry> & veh_poses) {

    for(const auto& pose: contacts.poses) {
        if(!hasContact(pose.header.frame_id, pose)) {
            addContact(pose.header.frame_id, pose);
            Contact & contact = mission_contacts.back();

            node_proxy_->log_info("Requesting move to %s at (%.2f, %.2f)", contact.name.c_str(), contact.pose.pose.position.x, contact.pose.pose.position.y);
            auto investigateTask = new MoveTo(this, my_name, -1, nullptr, 0.0, 0.0, toi_params_.approach_dist);
            investigateTask->setGoalByContact(contact);
            investigateTask->is_preemptable = false;
            addTask(investigateTask, PriorityType::PREEMPT);

            const int encircle_task_id = obj_detection_cnt--;
            auto encircleTask = new Encircle(this, my_name, encircle_task_id, contact.pose, toi_params_);
            encircleTask->is_preemptable = false;
            addTask(encircleTask, PriorityType::PREEMPT);

            // Get closest vehicle from vehh_poses
            double min_dist = std::numeric_limits<double>::max();
            std::string overwatch_veh;
            for(const auto& veh_pose: veh_poses) {
                if(veh_pose.first == my_name){
                    continue;
                }
                double dist = PosePlanarDistanceSq(veh_pose.second.pose.pose.position, contact.pose.pose.position);
                if(dist < min_dist) {
                    min_dist = dist;
                    overwatch_veh = veh_pose.first;
                }
            }

            if(overwatch_veh.empty()){
                node_proxy_->log_info("Could not find overwatch vehicle");
            }else{
                std::ostringstream stream;
                stream << my_name << "," << -1 << "," << "OVERWATCH," << overwatch_veh << "," << encircle_task_id;
                publishCommStr(stream.str());
            }
        }
    }
}

void MissionManager::handleOverwatch(const avt_341::msg::Communication & msg){

  MissionPoint mp = getClosestOverwatch();
  if(!mp.name.empty()){
    node_proxy_->log_info("Moving to overwatch %s at (%.2f, %.2f)", mp.name.c_str(), mp.pos_x, mp.pos_y);

    auto overwatchTask = new MoveTo(this, my_name, -1);
    overwatchTask->setGoalByMissionPoint(mp.name);
    overwatchTask->is_preemptable = false;
    addTask(overwatchTask, PriorityType::PREEMPT);

    auto waitTask = new WaitUntilComplete(this, my_name, -1, msg.sender_name, msg.target_msg_id);
    waitTask->is_preemptable = false;
    addTask(waitTask, PriorityType::PREEMPT);
  }else{
    node_proxy_->log_info("No overwatch found to investigate contact");
  }

}

MissionPoint MissionManager::getClosestOverwatch(){
  MissionPoint mp_out;
  double min_dist_sq = std::numeric_limits<double>::max();
  for(auto & mp : overwatch_positions){
    double dx = odometry.pose.pose.position.x - mp.pos_x;
    double dy = odometry.pose.pose.position.y - mp.pos_y;
    double dist_sq = dx*dx + dy*dy;
    if(dist_sq < min_dist_sq){
      min_dist_sq = dist_sq;
      mp_out = mp;
    }
  }
  return mp_out;
}

bool MissionManager::isMsgForSelf(const avt_341::msg::Communication & msg) {
  return msg.type == "TASK_COMPLETE" || msg.type == "ARRIVE" || (msg.type == "FORM" && FormationDefinition::vehicleInFormation(msg, my_name)) || msg.receiver_name == my_name;
}

void MissionManager::handleFormationRequest(avt_341::msg::Communication msg) {

    if(!FormationDefinition::vehicleInFormation(msg, my_name)){
      node_proxy_->log_info("Ignoring formation request. Not for me.");
      return;
    }

    MissionPoint mp;
    if(!getMissionPoint(mp, msg.objective_name)){
      node_proxy_->log_warning("Could not find mission point %s associated with formation.", msg.objective_name.c_str());
      return;
    }
    msg.receiver_name = my_name;
    auto formation_def = new FormationDefinition(msg, mp, formation_params);
    if(formation_def->isLeader() || formation_def->formationAtGoal()){
        // handle objective
        handleMoveTo(msg, formation_def->formation_status.x_offset, formation_def->formation_status.y_offset, formation_def);
    } else if(formation_def->isFollowing()) {
        Follow* followTask = new Follow(this, msg.sender_name, msg.msg_id, formation_def);
        addTask(followTask, msg.priority_type);
    }

    // handle set speed
    handleSetSpeed(msg);
}

void MissionManager::handleAcknowledge(const avt_341::msg::Communication & msg) {
    // <sender>,<msg_id>,ACK,<orig_msg_sender>,<orig_msg_id>
    if(msg.original_sender == my_name) {
        node_proxy_->log_info("%s acknowledged my msg #%s", msg.sender_name.c_str(), msg.original_msg_id.c_str());
    }
}

// <sender>,<msg_id>,ARRIVE,<objective>
void MissionManager::handleArrive(const avt_341::msg::Communication & msg) {
    // If tracking, update mission tracker
    arrivals_.push_back(msg);
}

// <sender>,<msg_id>,TASK_COMPLETE,<orig_msg_sender>,<orig_msg_id>
void MissionManager::handleTaskComplete(const avt_341::msg::Communication & msg) {
    // If tracking, mark complete
//    if(msg.original_sender == my_name) {
//        node_proxy_->log_info("%s has completed the assigned task from my msg #%s", msg.sender_name.c_str(), msg.original_msg_id.c_str());
//    }
    task_completions_.push_back(msg);
}

void MissionManager::handleMoveTo(const avt_341::msg::Communication & msg, double x_offset, double y_offset, FormationDefinition* formation_def) {
    // only applies if I'm the leader, otherwise decline
    if(msg.receiver_name == my_name) {
        MoveTo* moveTask = new MoveTo(this, msg.sender_name, msg.msg_id, formation_def, x_offset+msg.x_offset, y_offset+msg.y_offset, msg.distance);
        bool ret = moveTask->setGoalByMissionPoint(msg.objective_name);
        addTask(moveTask, msg.priority_type);
    } else {
        node_proxy_->log_info("Ignoring MoveTo (not for me)");
    }
}

void MissionManager::handleHold(const avt_341::msg::Communication & msg) {
    // handle request to wait

}

void MissionManager::handleSetSpeed(const avt_341::msg::Communication & msg) {
    // handle updated speed
    desired_speed = std::stof(msg.desired_speed);
//    node_proxy_->log_info("Setting desired speed to %.2f", desired_speed);
    avt_341::msg::Float64 speed_msg;
    speed_msg.data = desired_speed;
    speed_pub->publish(speed_msg);
}

void MissionManager::onGoalReached(const avt_341::msg::PoseStamped & pose){
  Task* task = currentTask();
  if(task != nullptr){
    task->onGoalReached(pose);
  }
}

void MissionManager::handleCancelTask(const avt_341::msg::Communication & msg){
  cancelTask(msg.target_msg_id, true);
  publishTaskCompletion(msg.sender_name, msg.msg_id);
}

void MissionManager::handleCancelAllTask(const avt_341::msg::Communication & msg){
  resetTaskList(true);
  publishTaskCompletion(msg.sender_name, msg.msg_id);
}

bool MissionManager::hasCompletedTask(const std::string & target_veh, int target_msg_id) const{
  return std::find_if(task_completions_.begin(), task_completions_.end(),
                   [&](const avt_341::msg::Communication & comm){return comm.sender_name == target_veh && comm.msg_id == target_msg_id;}) != task_completions_.end();
}

void MissionManager::publishArrival(const std::string & sender_name, const std::string & objective){
  std::ostringstream stream;
  if(add_name_id_to_msg_){
    stream << sender_name << ",-1," << "ARRIVE," << objective;
  }else{
    stream << "ARRIVE," << sender_name << "," << objective;
  }
  avt_341::msg::String comm_msg;
  comm_msg.data = stream.str();
  communication_pub->publish(comm_msg);
}

bool MissionManager::hasArrival(const std::string & target_veh, const std::string & objective) const{
  return std::find_if(arrivals_.begin(), arrivals_.end(),
                      [&](const avt_341::msg::Communication & comm){return comm.sender_name == target_veh && comm.objective_name == objective;}) != arrivals_.end();
}

} // namespace mission
} // namespace avt_341
