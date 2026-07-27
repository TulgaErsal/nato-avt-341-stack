#include <string>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#ifdef GTE_ROS_HUMBLE
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif


#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <rclcpp/rclcpp.hpp>
#include "avt_341_nav/node/node_utils.h"
#include "avt_341_nav/perception/normal_grid.h"
#include "avt_341_nav/avt_341_utils.h"
#include "avt_341_nav/node/tf_interface.h"


// Global variables
rclcpp::Node::SharedPtr node;
std::shared_ptr<avt_341_nav::node::TfInterface> tf;
std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> seg_grid_pub;
avt_341_nav::perception::NormalGrid grid;
nav_msgs::msg::Odometry current_pose;
bool odom_rcvd = false;

// ROS params
float grid_width, grid_height;
float grid_res, grid_llx, grid_lly, max_grid_width, max_grid_height, normal_threshold;
bool limit_grid_size;
std::string points_topic;

void CallbackNormCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Convert point cloud message
    pcl::PointCloud<pcl::PointNormal>::Ptr pc_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(*msg, *pc_normals);

    // Transform point cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr pc_fixed(new pcl::PointCloud<pcl::PointNormal>);
    if(msg->header.frame_id != "map") {
        geometry_msgs::msg::TransformStamped fixed_tf_msg = tf->lookup_transform("map", msg->header.frame_id, msg->header.stamp);
        Eigen::Affine3d fixed_tf = tf2::transformToEigen(fixed_tf_msg);
        pcl::transformPointCloudWithNormals(*pc_normals, *pc_fixed, fixed_tf.matrix());
    }
    else {
        *pc_fixed = *pc_fixed;
    }

    // Update normal grid
    grid.AddPoints(pc_fixed);
}

void OdometryCallback(nav_msgs::msg::Odometry::SharedPtr rcv_odom){
	current_pose = *rcv_odom;
	odom_rcvd = true;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("normal_segmentation_map_node");
    tf = std::make_shared<avt_341_nav::node::TfInterface>(node);

    // Load parameters
    avt_341_nav::node::get_parameter(node, "/grid_width", grid_width, 200.0f);
    avt_341_nav::node::get_parameter(node, "/grid_height", grid_height, 200.0f);
    avt_341_nav::node::get_parameter(node, "~grid_res", grid_res, 1.0f);
    avt_341_nav::node::get_parameter(node, "~grid_llx", grid_llx, -100.0f);
    avt_341_nav::node::get_parameter(node, "~grid_lly", grid_lly, -100.0f);
    avt_341_nav::node::get_parameter(node, "~limit_grid_size", limit_grid_size, false);
    avt_341_nav::node::get_parameter(node, "~max_grid_width", max_grid_width, 800.0f);
    avt_341_nav::node::get_parameter(node, "~max_grid_height", max_grid_height, 800.0f);
    avt_341_nav::node::get_parameter(node, "~normal_threshold", normal_threshold, 0.5f);
    avt_341_nav::node::get_parameter(node, "~points_topic", points_topic, std::string("avt_341/normals_cloud"));
    
    // Create publishers and subscribers
    auto norm_cloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>("avt_341/normals_cloud", 1, CallbackNormCloud);
    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>("avt_341/odometry",1, OdometryCallback);
    seg_grid_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("avt_341/normal_segmentation_grid", 1);

    // Setup grid
    grid.SetSize(grid_width,grid_height);
    grid.SetRes(grid_res);
	grid.SetCorner(grid_llx,grid_lly);
    grid.SetThreshold(normal_threshold);

    double rate = 10.0;
    rclcpp::Rate ros_rate(rate);
    while (rclcpp::ok()) {
        if (odom_rcvd) {
            // Publish grid
            nav_msgs::msg::OccupancyGrid grd;
            if (limit_grid_size)
                grd = grid.GetGrid(current_pose.pose.pose.position.x,current_pose.pose.pose.position.y,max_grid_width,max_grid_height);
            else
                grd = grid.GetGrid();
            grd.header.stamp = node->now();
            seg_grid_pub->publish(grd);
        }

        rclcpp::spin_some(node);
        ros_rate.sleep();
    }

    return 0;
}
