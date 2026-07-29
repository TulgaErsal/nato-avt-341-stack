/**
 * Publishes a occupancy grid from a geotiff dataset.
 * Publishes tf for map frame with respect to the epsg_XXXX frame (read from geotiff projection metadata).
 * Espects occupancy data in band #1 from 0->100 (free->lethal) with unknown = -1 or > 100.
 * 
 * Evan Vandermate (evanderm@mtu.edu)
 * Last Modified: 02/13/2024
*/
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <rclcpp/rclcpp.hpp>
#include "avt_341_nav/node/node_utils.h"
#include "avt_341_nav/node/tf_interface.h"
#include "avt_341_nav/perception/geotiff_dataset.h"
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>


void BuildOccupancyGrid(nav_msgs::msg::OccupancyGrid& grid, std::vector<double> data, int rows, int cols, double llx, double lly, float res) {
    grid.info.resolution = res;
    //rows and columns are swapped since the map is transposed
    grid.info.height = rows;
    grid.info.width = cols;
    grid.info.origin.position.x = llx;
    grid.info.origin.position.y = lly;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;
    grid.data.resize(rows*cols);

    int c = 0;
    for (double val : data)
    {
        grid.data[c++] = val <= 100 ? val : -1;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("avt_341_geotiff_map_publisher_node");
    auto tf = std::make_shared<avt_341_nav::node::TfInterface>(node);

    std::string tiff_path, map_frame, map_topic;
    int band;
    double map_origin_x, map_origin_y;
    float startup_delay;
    avt_341_nav::node::get_parameter(node, "~map_topic", map_topic, std::string("map"));
    avt_341_nav::node::get_parameter(node, "~tiff_path", tiff_path, std::string(""));
    avt_341_nav::node::get_parameter(node, "~map_frame", map_frame, std::string("geotiff"));
    avt_341_nav::node::get_parameter(node, "~band", band, 1);
    avt_341_nav::node::get_parameter(node, "~map_origin_x", map_origin_x, 0.0);
    avt_341_nav::node::get_parameter(node, "~map_origin_y", map_origin_y, 0.0);
    avt_341_nav::node::get_parameter(node, "~startup_delay", startup_delay, 5.0f);

    auto map_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic, 1);

    avt_341_nav::planning::Geotiff tiff(tiff_path);
    tiff.PrintInfo();
    std::vector<double> costmap = tiff.GetRasterBand(1);
    if (costmap.size() > 0)
    {
        //wait 2s to allow time for the rest of the stack to
        //start up since this message is only published once
        rclcpp::Rate rate(1.0/startup_delay);
        rate.sleep();

        // Publish occupancy grid
        nav_msgs::msg::OccupancyGrid grid;
        BuildOccupancyGrid(grid, costmap, tiff.rows, tiff.cols, map_origin_x, map_origin_y, tiff.resolution);
        grid.header.frame_id = map_frame;
        map_pub->publish(grid);

        // Publish transform to proj frame
        std::string projFrame = "epsg_" + tiff.GetProjection();
        if (projFrame.compare("epsg_") == 0) {  // No projection set, frame:geotiff = frame:map
            projFrame = "map";
            tf->publish_static_tf(projFrame, map_frame, geometry_msgs::msg::PoseStamped());
        }
        else {
            geometry_msgs::msg::PoseStamped tiff_pose;
            tf2::Matrix3x3 m(   tiff.transform[1],    tiff.transform[2],  0.0,
                                            tiff.transform[4],    tiff.transform[5],  0.0,
                                            0.0,                  0.0,                1.0   );
            m = m.inverse();
            tf2::Quaternion q;
            m.getRotation(q);
            tiff_pose.pose.position.x = tiff.transform[0];
            tiff_pose.pose.position.y = tiff.transform[3]-tiff.rows*tiff.resolution;   // Move origin to lower-left corner
            tiff_pose.pose.position.z = 0.0;
            tiff_pose.pose.orientation.x = q.x();
            tiff_pose.pose.orientation.y = q.y();
            tiff_pose.pose.orientation.z = q.z();
            tiff_pose.pose.orientation.w = q.w();
            tf->publish_static_tf(projFrame, map_frame, tiff_pose);
        }

        rclcpp::spin(node);
    }
    return 0;
}