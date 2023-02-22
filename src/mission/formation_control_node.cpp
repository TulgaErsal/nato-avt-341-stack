// ros includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
// local includes
#include "avt_341/avt_341_utils.h"
#include "avt_341/mission/formation_controller.h"

int main(int argc, char **argv){

    auto n = avt_341::node::init_node(argc,argv,"formation_controller");

    avt_341::node::Rate loop_rate(10);
    
    while(avt_341::node::ok()){


        n->spin_some();
        loop_rate.sleep();
    }
}
