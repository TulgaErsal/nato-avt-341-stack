/**
 * Simple local occupancy grid implementation
 * 
 * Evan Vandermate - evanderm@mtu.edu
*/
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/perception/local_occupancy_grid.h"


// Globals
std::shared_ptr<avt_341::node::NodeProxy> n;
std::shared_ptr<avt_341::perception::LocalOccupancyGrid> grid;
avt_341::msg::PointCloud2 points;
std::string local_frame, frame, cloud_topic;
float width, height, resolution, dilate_x, dilate_y, rate_hz;
bool dilate;

bool GetLocalPose(avt_341::msg::PoseStamped& pose_out) {
    avt_341::msg::PoseStamped local_pose;
    local_pose.header.stamp = n->get_stamp();
    local_pose.header.frame_id = local_frame;
    local_pose.pose.position.x = 0.0;
    local_pose.pose.position.y = 0.0;
    local_pose.pose.position.z = 0.0;
    if (!n->transform_pose(local_pose, pose_out, frame)) {
        return false;
    }
    return true;
}

void CloudCallback(avt_341::msg::PointCloud2Ptr rcv_points)
{
    // Transform cloud to map frame
    n->transform_cloud(*rcv_points, points, frame);
}

void UpdateGrid() {
    // Update map origin
    avt_341::msg::PoseStamped map_pose;
    if (!GetLocalPose(map_pose)) {
        return;
    }
    int origin_x = map_pose.pose.position.x - (int)(width/2.0);
    int origin_y = map_pose.pose.position.y - (int)(height/2.0);
    grid->UpdateOrigin(origin_x, origin_y);

    // Add obstacle points to grid
    if (points.data.size() > 0) {
        grid->AddPoints(points, dilate);
    }
}

int main(int argc, char** argv) {
    n = avt_341::node::init_node(argc, argv, "avt_341_local_occupancy_grid_node");
    n->initialize_tf_listener();

    n->get_parameter("~local_grid_frame", local_frame, std::string("base_link"));
    n->get_parameter("~global_grid_frame", frame, std::string("map"));
    n->get_parameter("~obstacle_points_topic", cloud_topic, std::string("avt_341/lidar_detector/cloud_clusters"));
    n->get_parameter("~local_grid_width", width, 20.0f);
    n->get_parameter("~local_grid_height", height, 20.0f);
    n->get_parameter("~local_grid_resolution", resolution, 0.1f);
    n->get_parameter("~dilate_local_grid", dilate, false);
    n->get_parameter("~local_grid_dilate_x", dilate_x, 1.0f);
    n->get_parameter("~local_grid_dilate_y", dilate_y, 1.0f);
    n->get_parameter("~local_grid_rate", rate_hz, 5.0f);

    grid = std::make_shared<avt_341::perception::LocalOccupancyGrid>(frame, width, height, resolution, dilate_x, dilate_y);

    auto occupancy_grid_pub = n->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/local_grid", 10);
    auto obstacle_points_sub = n->create_subscription<avt_341::msg::PointCloud2>(cloud_topic, 10, CloudCallback);

    avt_341::node::Rate rate(rate_hz);

    while (avt_341::node::ok()) {
        UpdateGrid();
        avt_341::msg::OccupancyGrid grd = grid->GetGrid();
        occupancy_grid_pub->publish(grd);

        n->spin_some();
        rate.sleep();
    }

    return 0;
}
