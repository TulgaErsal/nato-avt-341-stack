/**
 * \file data_acquisition_node.cpp
 * Calculate and record CG vehicle dynamics.
 * 
 * \author Evan Vandermate
 *
 * \contact evanderm@mtu.edu
 * 
 * \date 01/23/2024
 */
#include <cmath>

// ROS includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "std_msgs/Duration.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"


std_msgs::Duration time_elapsed;

int main(int argc, char *argv[]){

    // Init node
    auto n = avt_341::node::init_node(argc, argv, "data_acquisition_node");
    n->initialize_tf_listener();

    // Parameters
    double local_origin_x, local_origin_y;
    std::string frame_world, frame_map, frame_cg;
    float vel_window, rate;
    n->get_parameter("/map_origin_x", local_origin_x, 0.0);
    n->get_parameter("/map_origin_y", local_origin_y, 0.0);
    n->get_parameter("~world_frame", frame_world, std::string("nad83"));
    n->get_parameter("~map_frame", frame_map, std::string("map"));
    n->get_parameter("~vehicle_cg_frame", frame_cg, std::string("vbox_link"));
    n->get_parameter("~velocity_averaging_window", vel_window, 0.2f);
    n->get_parameter("~rate", rate, 60.0f);

    // Create publishers and subscribers
    auto time_pub = n->create_publisher<std_msgs::Duration>("avt_341/elapsed_time", 10);
    auto dist_pub = n->create_publisher<avt_341::msg::Float64>("avt_341/vehicle_cg/dist_travelled", 10);
    auto avg_speed_pub = n->create_publisher<avt_341::msg::Float64>("avt_341/vehicle_cg/avg_speed", 10);
    auto cg_pos_pub = n->create_publisher<avt_341::msg::PoseStamped>("avt_341/vehicle_cg/pos", 10);
    auto cg_vel_pub = n->create_publisher<avt_341::msg::TwistStamped>("avt_341/vehicle_cg/vel", 10);
    auto cg_accel_pub = n->create_publisher<avt_341::msg::AccelStamped>("avt_341/vehicle_cg/accel", 10);

    // Timestep variables
    int64_t loop_count = 0;
    avt_341::msg::Float64 dist_travelled;
    avt_341::msg::Float64 avg_speed;
    avt_341::msg::Duration vel_duration = avt_341::node::make_duration(vel_window);
    avt_341::msg::Time last_t;
    avt_341::msg::PoseStamped last_cg_pos;
    avt_341::msg::TwistStamped last_cg_vel;

    // MAINLOOP
    avt_341::msg::Time t_start = n->get_stamp();
    avt_341::node::Rate rosrate(rate);
    while (avt_341::node::ok())
    {
        // Current elapsed time
        avt_341::msg::Time t_now = n->get_stamp();
        time_elapsed.data = t_now - t_start;
        time_pub->publish(time_elapsed);

        // Get CG transform
        avt_341::msg::TransformStamped tfs_ref = n->lookup_transform(frame_world, frame_cg);

        // Calculate CG dynamics
        if (tfs_ref.header.stamp.toSec() > 1e-3f)
        {
            // Publish CG position
            avt_341::msg::PoseStamped cg_pos;
            cg_pos.header = tfs_ref.header;
            cg_pos.pose.orientation = tfs_ref.transform.rotation;
            cg_pos.pose.position.x = tfs_ref.transform.translation.x;
            cg_pos.pose.position.y = tfs_ref.transform.translation.y;
            cg_pos.pose.position.z = tfs_ref.transform.translation.z;
            cg_pos_pub->publish(cg_pos);

            // Lookup old tf
            avt_341::msg::Time old_stamp = tfs_ref.header.stamp - vel_duration;
            avt_341::msg::TransformStamped tfs_old = n->lookup_transform(frame_cg, old_stamp, frame_cg, tfs_ref.header.stamp, frame_map);
            
            // Calculate twist
            float dt_window = 1.0 / vel_window;
            avt_341::msg::Vector3 trans = tfs_old.transform.translation;
            avt_341::msg::Quaternion rot = tfs_old.transform.rotation;
            tf2::Quaternion q_rot(rot.x, rot.y, rot.z, rot.w);
            tf2::Matrix3x3 m_rot(q_rot);
            double roll, pitch, yaw;
            m_rot.getRPY(roll, pitch, yaw);

            // Publish CG velocity
            avt_341::msg::TwistStamped cg_vel;
            cg_vel.header = tfs_ref.header;
            cg_vel.header.frame_id = frame_cg;
            cg_vel.twist.linear.x = trans.x * dt_window;
            cg_vel.twist.linear.y = trans.y * dt_window;
            cg_vel.twist.linear.z = trans.z * dt_window;
            cg_vel.twist.angular.x = roll * dt_window;
            cg_vel.twist.angular.y = pitch * dt_window;
            cg_vel.twist.angular.z = yaw * dt_window;
            cg_vel_pub->publish(cg_vel);

            // Publish CG average speed
            avg_speed.data += (cg_vel.twist.linear.x - avg_speed.data) / (double)(++loop_count);
            avg_speed_pub->publish(avg_speed);

            if (last_t.toSec() > 1e-3)
            {
                // Publish distance travelled
                avt_341::msg::Point pos = cg_pos.pose.position;
                avt_341::msg::Point last_pos = last_cg_pos.pose.position;
                double dx_sqr = pow((double)pos.x - (double)last_pos.x,2);
                double dy_sqr = pow((double)pos.y - (double)last_pos.y,2);
                double dz_sqr = pow((double)pos.z - (double)last_pos.z,2);
                if (cg_vel.twist.linear.x > 0.2)    // Check that longitudinal velocity is significant
                {
                    dist_travelled.data += sqrt(dx_sqr + dy_sqr + dz_sqr);
                }
                dist_pub->publish(dist_travelled);

                // Publish CG acceleration (estimate from velocity)
                double dt = (t_now - last_t).toSec();
                avt_341::msg::AccelStamped cg_accel;
                cg_accel.header = tfs_ref.header;
                cg_accel.header.frame_id = frame_cg;
                cg_accel.accel.linear.x = (cg_vel.twist.linear.x - last_cg_vel.twist.linear.x) / dt;
                cg_accel.accel.linear.y = (cg_vel.twist.linear.y - last_cg_vel.twist.linear.y) / dt;
                cg_accel.accel.linear.z = (cg_vel.twist.linear.z - last_cg_vel.twist.linear.z) / dt;
                cg_accel.accel.angular.x = (cg_vel.twist.angular.x - last_cg_vel.twist.angular.x) / dt;
                cg_accel.accel.angular.y = (cg_vel.twist.angular.y - last_cg_vel.twist.angular.y) / dt;
                cg_accel.accel.angular.z = (cg_vel.twist.angular.z - last_cg_vel.twist.angular.z) / dt;
                cg_accel_pub->publish(cg_accel);
            }

            // Update timestep variables
            last_t = t_now;
            last_cg_pos = cg_pos;
            last_cg_vel = cg_vel;

        }

        // Loop rate control
        n->spin_some();
        rosrate.sleep();
    }
}