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
#include <vector>

// ROS includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include <avt_341/data_acquisition_params_service.hpp>


avt_341::msg::Twist cmd_vel;
double vel;
bool cmd_rcvd;
bool vel_rcvd;

void cmd_vel_callback(avt_341::msg::TwistPtr msg) {
    cmd_vel = *msg;
    cmd_rcvd = true;
}

void speed_callback(avt_341::msg::Float64Ptr rcv_speed) {
	vel = rcv_speed->data;
    vel_rcvd = true;
}

int main(int argc, char *argv[]){
    // Init node
    auto n = avt_341::node::init_node(argc, argv, "data_acquisition_node");
    avt_341::params::data_acquisition::ParamsListener param_listener(n->get_raw_node());
    const auto params = param_listener.get_params();
    n->initialize_tf_listener();

    const int accel_samples =
        static_cast<int>(params.accel_averaging_samples);

    // Create publishers and subscribers
    auto time_pub = n->create_publisher<avt_341::msg::DurationMsg>("avt_341/elapsed_time", 10);
    auto dist_pub = n->create_publisher<avt_341::msg::Float64>("avt_341/vehicle_cg/dist_travelled", 10);
    auto avg_speed_pub = n->create_publisher<avt_341::msg::Float64>("avt_341/vehicle_cg/avg_speed", 10);
    auto cg_pos_pub = n->create_publisher<avt_341::msg::PoseStamped>("avt_341/vehicle_cg/pos", 10);
    auto cg_vel_pub = n->create_publisher<avt_341::msg::TwistStamped>("avt_341/vehicle_cg/vel", 10);
    auto cg_accel_pub = n->create_publisher<avt_341::msg::AccelStamped>("avt_341/vehicle_cg/accel", 10);
    auto cg_lat_g_pub = n->create_publisher<avt_341::msg::Float64>("avt_341/vehicle_cg/lateral_g", 10);
    auto cmd_vel_sub = n->create_subscription<avt_341::msg::Twist>("avt_341/cmd_vel",1,cmd_vel_callback);
    auto speed_sub = n->create_subscription<avt_341::msg::Float64>("avt_341/forward_speed",1,speed_callback);

    // Timestep variables
    int64_t loop_count = 0;
    avt_341::msg::Float64 dist_travelled;
    avt_341::msg::Float64 avg_speed;
    avt_341::msg::Duration vel_duration =
        avt_341::node::make_duration(params.velocity_averaging_window);
    avt_341::msg::Time last_t;
    avt_341::msg::PoseStamped last_cg_pos;
    avt_341::msg::TwistStamped last_cg_vel;
    cmd_rcvd = false;
    vel_rcvd = false;
    
    // Acceleration averaging values
    std::vector<std::vector<float>> accel_vals;
    std::vector<float> accel_frame;
    accel_frame.resize(6, 0.0f);  // linear (x, y, z), angular (x, y, z) -> 6 values
    accel_vals.resize(accel_samples,accel_frame);
    int i_accel = 0;

    // MAINLOOP
    avt_341::msg::Time t_start = n->get_stamp();
    avt_341::node::Rate rosrate(params.daq_rate);
    while (avt_341::node::ok())
    {
        // Current elapsed time
        avt_341::msg::Time t_now = n->get_stamp();
        avt_341::msg::Duration time_elapsed = t_now - t_start;
        time_pub->publish(time_elapsed);

        // Publish lateral acceleration
        if (cmd_rcvd && vel_rcvd) {
            avt_341::msg::Float64 lat_accel;
            lat_accel.data =
                (vel * vel) * tan(cmd_vel.angular.z) / params.wheelbase / 9.81;
            cg_lat_g_pub->publish(lat_accel);
        }

        // Get CG transform
        avt_341::msg::TransformStamped tfs_ref =
            n->lookup_transform(params.world_frame, params.vehicle_cg_frame);

        // Calculate CG dynamics
        if (avt_341::node::seconds_from_header(tfs_ref.header) > 1e-3f)
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
            avt_341::msg::Time old_stamp = avt_341::msg::Time(tfs_ref.header.stamp) - vel_duration;
            avt_341::msg::TransformStamped tfs_old = n->lookup_transform(
                params.vehicle_cg_frame, old_stamp, params.vehicle_cg_frame,
                tfs_ref.header.stamp, params.map_frame);
            
            // Calculate twist
            const double dt_window = 1.0 / params.velocity_averaging_window;
            avt_341::msg::Vector3 trans = tfs_old.transform.translation;
            avt_341::msg::Quaternion rot = tfs_old.transform.rotation;
            tf2::Quaternion q_rot(rot.x, rot.y, rot.z, rot.w);
            tf2::Matrix3x3 m_rot(q_rot);
            double roll, pitch, yaw;
            m_rot.getRPY(roll, pitch, yaw);

            // Publish CG velocity
            avt_341::msg::TwistStamped cg_vel;
            cg_vel.header = tfs_ref.header;
            cg_vel.header.frame_id = params.vehicle_cg_frame;
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

            if (avt_341::node::seconds_from_time(last_t) > 1e-3)
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

                // Calculate current acceleration estimate
                double dt = avt_341::node::seconds_from_time(t_now - last_t);
                if (i_accel >= accel_samples) {
                    i_accel = 0;
                }
                accel_vals[i_accel][0] = (cg_vel.twist.linear.x - last_cg_vel.twist.linear.x) / dt;
                accel_vals[i_accel][1] = (cg_vel.twist.linear.y - last_cg_vel.twist.linear.y) / dt;
                accel_vals[i_accel][2] = (cg_vel.twist.linear.z - last_cg_vel.twist.linear.z) / dt;
                accel_vals[i_accel][3] = (cg_vel.twist.angular.x - last_cg_vel.twist.angular.x) / dt;
                accel_vals[i_accel][4] = (cg_vel.twist.angular.y - last_cg_vel.twist.angular.y) / dt;
                accel_vals[i_accel][5] = (cg_vel.twist.angular.z - last_cg_vel.twist.angular.z) / dt;
                i_accel++;

                // Average acceleration estimates
                accel_frame.clear();
                accel_frame.resize(6, 0.0f);
                for (int i = 0; i < accel_vals.size(); i++) {
                    for (int j = 0; j < accel_frame.size(); j++) {
                        accel_frame[j] += accel_vals[i][j];
                    }
                }
                for (int j = 0; j < accel_frame.size(); j++) {
                    accel_frame[j] /= accel_vals.size();
                }

                // Publish averaged acceleration
                avt_341::msg::AccelStamped cg_accel;
                cg_accel.header = tfs_ref.header;
                cg_accel.header.frame_id = params.vehicle_cg_frame;
                cg_accel.accel.linear.x = accel_frame[0];
                cg_accel.accel.linear.y = accel_frame[1];
                cg_accel.accel.linear.z = accel_frame[2];
                cg_accel.accel.angular.x = accel_frame[3];
                cg_accel.accel.angular.y = accel_frame[4];
                cg_accel.accel.angular.z = accel_frame[5];
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
