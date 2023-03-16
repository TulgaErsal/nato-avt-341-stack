// ros includes
#include "avt_341/node/node_proxy.h"
// local includes
#include "avt_341/mission/mission_manager.h"

avt_341::msg::Communication rcvd_msg;
bool message_rcvd = false;

avt_341::msg::String my_name;

void CommunicationCallback(avt_341::msg::CommunicationPtr msg) {
    ROS_INFO("%s Mission Manager received communication", ros::this_node::getName().c_str()); 
    rcvd_msg = *msg;
    message_rcvd = true;
}

int main(int argc, char **argv) {

    // initialize the node
    auto nh = avt_341::node::init_node(argc, argv, "mission_manager");
    avt_341::node::Rate loop_rate(10);

    // set up subscriptions
    auto communication_sub = nh->create_subscription<avt_341::msg::Communication>("avt_341/recv_comms", 10, CommunicationCallback);
    
    // create the publishers, each vehicle publishes a path and a desired speed
    //auto path_pub = n->create_publisher<avt_341::msg::Path>("avt_341/global_path", 10);
    
    // load the parameters
    nh->get_parameter("~name", my_name.data, std::string("AGV1"));

    // create the controller and set the parameters loaded from the launch file
    avt_341::mission::MissionManager mgr;

    // start the loop
    while(avt_341::node::ok()){
        if(message_rcvd) {
            ROS_INFO("%s Handling message", ros::this_node::getName().c_str());
            message_rcvd = false;
        }

        nh->spin_some();
        loop_rate.sleep();
    }
}