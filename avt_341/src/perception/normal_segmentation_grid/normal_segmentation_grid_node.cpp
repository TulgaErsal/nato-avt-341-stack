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


#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/perception/normal_grid.h"
#include "avt_341/avt_341_utils.h"


// Global variables
std::shared_ptr<avt_341::node::NodeProxy> node;
std::shared_ptr<avt_341::node::Publisher<avt_341::msg::OccupancyGrid>> seg_grid_pub;
avt_341::perception::NormalGrid grid;
avt_341::msg::Odometry current_pose;
bool odom_rcvd = false;

// ROS params
float grid_width, grid_height;
float grid_res, grid_llx, grid_lly, max_grid_width, max_grid_height, normal_threshold;
bool limit_grid_size;
std::string points_topic;

void CallbackNormCloud(avt_341::msg::PointCloud2Ptr msg) {
    // Convert point cloud message
    pcl::PointCloud<pcl::PointNormal>::Ptr pc_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(*msg, *pc_normals);

    // Transform point cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr pc_fixed(new pcl::PointCloud<pcl::PointNormal>);
    if(msg->header.frame_id != "map") {
        avt_341::msg::TransformStamped fixed_tf_msg = node->lookup_transform("map", msg->header.frame_id, msg->header.stamp);
        Eigen::Affine3d fixed_tf = tf2::transformToEigen(fixed_tf_msg);
        pcl::transformPointCloudWithNormals(*pc_normals, *pc_fixed, fixed_tf.matrix());
    }
    else {
        *pc_fixed = *pc_fixed;
    }

    // Update normal grid
    grid.AddPoints(pc_fixed);
}

void OdometryCallback(avt_341::msg::OdometryPtr rcv_odom){
	current_pose = *rcv_odom;
	odom_rcvd = true;
}

int main(int argc, char* argv[]) {
    node = avt_341::node::init_node(argc, argv, "normal_segmentation_map_node");
    node->initialize_tf_listener();

    // Load parameters
    node->get_parameter("/grid_width", grid_width, 200.0f);
    node->get_parameter("/grid_height", grid_height, 200.0f);
    node->get_parameter("~grid_res", grid_res, 1.0f);
    node->get_parameter("~grid_llx", grid_llx, -100.0f);
    node->get_parameter("~grid_lly", grid_lly, -100.0f);
    node->get_parameter("~limit_grid_size", limit_grid_size, false);
    node->get_parameter("~max_grid_width", max_grid_width, 800.0f);
    node->get_parameter("~max_grid_height", max_grid_height, 800.0f);
    node->get_parameter("~normal_threshold", normal_threshold, 0.5f);
    node->get_parameter("~points_topic", points_topic, std::string("avt_341/normals_cloud"));
    
    // Create publishers and subscribers
    auto norm_cloud_sub = node->create_subscription<avt_341::msg::PointCloud2>("avt_341/normals_cloud", 1, CallbackNormCloud);
    auto odom_sub = node->create_subscription<avt_341::msg::Odometry>("avt_341/odometry",1, OdometryCallback);
    seg_grid_pub = node->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/normal_segmentation_grid", 1);

    // Setup grid
    grid.SetSize(grid_width,grid_height);
    grid.SetRes(grid_res);
	grid.SetCorner(grid_llx,grid_lly);
    grid.SetThreshold(normal_threshold);

    double rate = 10.0;
    avt_341::node::Rate ros_rate(rate);
    while (avt_341::node::ok()) {
        if (odom_rcvd) {
            // Publish grid
            avt_341::msg::OccupancyGrid grd;
            if (limit_grid_size)
                grd = grid.GetGrid(current_pose.pose.pose.position.x,current_pose.pose.pose.position.y,max_grid_width,max_grid_height);
            else
                grd = grid.GetGrid();
            grd.header.stamp = node->get_stamp();
            seg_grid_pub->publish(grd);
        }

        node->spin_some();
        ros_rate.sleep();
    }

    return 0;
}
