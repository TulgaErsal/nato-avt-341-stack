// dual_param_publisher.cpp
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_param_publisher");
    ros::NodeHandle nh;

    // パラメータサーバーからpublishする2つの値を取得
    int publish_value1, publish_value2;
    nh.param("publish_value1", publish_value1, 3); // デフォルト値10
    nh.param("publish_value2", publish_value2, 5); // デフォルト値20

    ros::Publisher pub1 = nh.advertise<std_msgs::Int32>("mpc/numColPoints", 1000);
    ros::Publisher pub2 = nh.advertise<std_msgs::Float64>("mpc/predictionTimeHorizon", 1000);
    ros::Rate loop_rate(100); // 10Hz

    while (ros::ok())
    {
        std_msgs::Int32 msg1;
        msg1.data = publish_value1;
        std_msgs::Float64 msg2;
        msg2.data = publish_value2;
        
        pub1.publish(msg1);
        pub2.publish(msg2);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
