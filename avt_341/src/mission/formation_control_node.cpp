// ros includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
// local includes
#include "avt_341/avt_341_utils.h"
#include "avt_341/mission/formation_controller.h"

avt_341::msg::Odometry odom;
avt_341::msg::Odometry ldr_odom;
avt_341::msg::FollowerStatus status;
avt_341::msg::String my_name;
bool odom_rcvd = false;
bool ldr_odom_rcvd = false;
bool status_rcvd = false;
bool is_leader = false;
bool reset_called = false;
std::shared_ptr<avt_341::node::NodeProxy> n = nullptr;

void LogStatusUpdate(){
    n->log_info("%s State: odom %d, ldr odom %d, follow_status %d, is_leader %d", my_name.data.c_str(), odom_rcvd, ldr_odom_rcvd, status_rcvd, is_leader);
}

void OdometryCallback(avt_341::msg::OdometryPtr rcv_odom){
    odom = *rcv_odom;
    if(!odom_rcvd){
        odom_rcvd = true;
        LogStatusUpdate();
    }
}

void LeaderOdometryCallback(avt_341::msg::OdometryPtr rcv_odom){
    ldr_odom = *rcv_odom;
    if(!ldr_odom_rcvd){
        ldr_odom_rcvd = true;
        LogStatusUpdate();
    }
}

void StatusCallback(avt_341::msg::FollowerStatusPtr rcv_status){
    status = *rcv_status;
    if(!status_rcvd){
        status_rcvd = true;
        LogStatusUpdate();
    }
}

void ResetCallback(avt_341::msg::Int32Ptr msg){
  reset_called = true;
}

int main(int argc, char **argv){

    // create the node
    n = avt_341::node::init_node(argc,argv,"formation_controller");

    // set up subscriptions
    // all vehicles subscribe to their own odometry and the leader odometry separately
    auto odometry_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 10, OdometryCallback);
    auto leader_odom_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/leader_odometry", 10, LeaderOdometryCallback);
    auto status_sub = n->create_subscription<avt_341::msg::FollowerStatus>("avt_341/follower_status", 10, StatusCallback);
    
    // create the publishers, each vehicle publishes a path and a desired speed
    auto path_pub = n->create_publisher<avt_341::msg::Path>("avt_341/global_path", 10);
    auto gptoggle_pub = n->create_publisher<avt_341::msg::Int32>("avt_341/gp_toggle", 10);
    auto goal_pub = n->create_publisher<avt_341::msg::PoseStamped>("avt_341/goal_pose", 10);
    auto reset_sub = n->create_subscription<avt_341::msg::Int32>("avt_341/reset", 10, ResetCallback);

    avt_341::msg::Int32 gp_toggle;

    // load the parameters
    // specify if the vehicle is the leader
    n->get_parameter("~is_leader", is_leader, false);

    // parameter of the formation controller
    float dist_gain = 1.0f;
    n->get_parameter("~follower_dist_gain", dist_gain, 1.0f);

    // another parameter of the formation controller
    float path_point_dist = 1.0f;
    bool use_breadcrumbs, x_offset_on_path;
    std::string fsc_type;
    n->get_parameter("~global_path_point_dist", path_point_dist, 1.0f);
    n->get_parameter("~use_leader_breadcrumbs", use_breadcrumbs, true);
    n->get_parameter("~name", my_name.data, std::string("AGV1"));
    n->get_parameter("~fsc_type", fsc_type, FormationSpeedControlType::SPEED_UP_FOLLOWER);
    n->get_parameter("~x_offset_on_path", x_offset_on_path, false);

    bool is_speed_up_follower = fsc_type == FormationSpeedControlType::SPEED_UP_FOLLOWER;
    auto speed_pub = is_speed_up_follower ? n->create_publisher<avt_341::msg::Float64>("avt_341/desired_speed", 10) : nullptr;

    // create the controller and set the parameters loaded from the launch file
    avt_341::mission::FormationController controller;
    controller.SetGlobalPathPointsDist(path_point_dist);
    controller.SetFollowerDistGain(dist_gain);
    controller.SetXOffsetOnPath(x_offset_on_path);

    // set the node loop ratre to 10 Hz
    avt_341::node::Rate loop_rate(10);
    int loop_cnt = 0;
    LogStatusUpdate();

    // start the loop
    while(avt_341::node::ok()){

        if(reset_called){
          n->log_info("Resetting node");
          status_rcvd = false;
          controller.Reset();
          reset_called = false;
        }

        // Update leader status - gp_toggle should probably be in manager. Leaving it here for now as it would break Tamer's work. 
        if(status_rcvd) {
            // if not currently leader and status is not telling me to use the leader, I'm the leader
            if(!is_leader) {
                if(!status.use_leader) {
                    is_leader = true;
                    controller.ClearDesiredGlobalPath();
                    gp_toggle.data = 1;
                    gptoggle_pub->publish(gp_toggle);
                    LogStatusUpdate();
                }
            } else {
                // if I am the leader and status is telling me to use the leader, I'm the follower
                if(status.use_leader) {
                    is_leader = false;
                    controller.ClearDesiredGlobalPath();
                    if(use_breadcrumbs) {
                        gp_toggle.data = 0;
                        gptoggle_pub->publish(gp_toggle);
                    }
                    LogStatusUpdate();
                }
            }
        }


        if ( (odom_rcvd && ldr_odom_rcvd && status_rcvd && !is_leader) ){
            //n->log_info("%s Formation Control: Updating follower controller.", my_name.data.c_str());
            // update the controller IF all the required messages have been received
            controller.Update(ldr_odom, odom, status);
            // publish the controller state
            //n->log_info("%s Formation Control: Publishing speed and path %0.2f", my_name.data.c_str(), controller.GetSpeed().data);

            if(is_speed_up_follower){
                if(controller.GetSpeed().data > 10.0) {
                    avt_341::msg::Float64 spd;
                    spd.data = 10.0;
                    speed_pub->publish(spd);
                } else {
                    speed_pub->publish(controller.GetSpeed());
                }
            }

            auto follower_path = controller.GetPath();
            if(use_breadcrumbs){
                path_pub->publish(follower_path);
            }else if(loop_cnt % 10 == 0 && !follower_path.poses.empty()){
                avt_341::msg::PoseStamped goal;
                goal.header.frame_id = "map";
                goal.header.stamp = n->get_stamp();
                goal.pose = follower_path.poses.back().pose;
                goal_pub->publish(goal);
            }
        }

        loop_cnt += 1;
        n->spin_some();
        loop_rate.sleep();
    }
}
