#include <string>
#include <memory>
#include <iostream>
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"


// Global variables
std::shared_ptr<avt_341::node::NodeProxy> node;
std::shared_ptr<avt_341::node::Publisher<avt_341::msg::PointCloud2>> cloud_pub;
Eigen::Vector4f roi_max_point, roi_min_point, body_max_point, body_min_point;

// ROS params
std::string pc_topic, robot_base_link, fixed_frame;
float roi_max_x, roi_max_y, roi_max_z, roi_min_x, roi_min_y, roi_min_z;
float body_max_x, body_max_y, body_max_z, body_min_x, body_min_y, body_min_z;
float obstacle_scale;

pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const Eigen::Vector4f& min_pt, const Eigen::Vector4f& max_pt, const Eigen::Vector4f& body_min_pt, const Eigen::Vector4f& body_max_pt)
{
    // Cropping the ROI
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> region(true);
    region.setMin(min_pt);
    region.setMax(max_pt);
    region.setInputCloud(cloud);
    region.filter(*cloud_roi);

    // Removing the car roof region
    std::vector<int> indices;
    pcl::CropBox<pcl::PointXYZ> roof(true);
    roof.setMin(body_min_pt);
    roof.setMax(body_max_pt);
    roof.setInputCloud(cloud_roi);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (auto& point : indices)
    inliers->indices.push_back(point);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_roi);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_roi);

    // const auto end_time = std::chrono::steady_clock::now();
    // const auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    // std::cout << "filtering took " << elapsed_time.count() << " milliseconds" << std::endl;

    return cloud_roi;
}

void pclComputeNorms(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out, float scale)
{
	// Create a search tree, use KDTreee for non-organized data.
	pcl::search::Search<pcl::PointXYZ>::Ptr tree;
	tree.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));

	// Set the input pointcloud for the search tree
	tree->setInputCloud(cloud_in);

	// Compute normals using both small and large scales at each point
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
	ne.setInputCloud(cloud_in);
	ne.setSearchMethod(tree);

	/**
	 * NOTE: setting viewpoint is very important, so that we can ensure
	 * normals are all pointed in the same direction!
	 */
	ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

	// calculate normals
	copyPointCloud(*cloud_in, *cloud_out);
	ne.setRadiusSearch(scale);
	ne.compute(*cloud_out);
}

void callback_cloud(avt_341::msg::PointCloud2Ptr msg) {
    // Transform point cloud
    avt_341::msg::PointCloud2 lidar_points_fixed;
    if(msg->header.frame_id != robot_base_link)
    {
        if (!node->transform_cloud(*msg, lidar_points_fixed, robot_base_link)) {
            node->log_warning("Unable to transform pointcloud from %s -> %s",msg->header.frame_id.c_str(),robot_base_link.c_str());
            return;
        }
    }
    else
    {
        lidar_points_fixed = *msg;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(lidar_points_fixed, *raw_cloud);

    // Downsampling, ROI, and removing the car roof
    auto filtered_cloud = filterCloud(raw_cloud, roi_min_point, roi_max_point, body_min_point, body_max_point);

    if(filtered_cloud->size() < 10) return;

    // Estimate cloud normals
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pclComputeNorms(filtered_cloud, normals_cloud, obstacle_scale);

    avt_341::msg::PointCloud2 normals_cloud_msg;
    pcl::toROSMsg(*normals_cloud, normals_cloud_msg);
    normals_cloud_msg.header = lidar_points_fixed.header;
    cloud_pub->publish(normals_cloud_msg);
}

int main(int argc, char* argv[]) {
    node = avt_341::node::init_node(argc, argv, "lidar_normal_estimation_node");
    node->initialize_tf_listener();

    // Load parameters
    node->get_parameter("~pc_topic", pc_topic, std::string("avt_341/points"));
    node->get_parameter("~robot_base_link", robot_base_link, std::string("mrzr/base_link"));
    node->get_parameter("~fixed_frame", fixed_frame, std::string("map"));
    node->get_parameter("~roi_max_x", roi_max_x,  70.0f);
    node->get_parameter("~roi_max_y", roi_max_y,  30.0f);
    node->get_parameter("~roi_max_z", roi_max_z,  3.0f);
    node->get_parameter("~roi_min_x", roi_min_x,  -5.0f);
    node->get_parameter("~roi_min_y", roi_min_y,  -30.0f);
    node->get_parameter("~roi_min_z", roi_min_z,  -2.5f);
    node->get_parameter("~body_max_x", body_max_x,  0.3f);
    node->get_parameter("~body_max_y", body_max_y,  0.8f);
    node->get_parameter("~body_max_z", body_max_z,  2.0f);
    node->get_parameter("~body_min_x", body_min_x,  -2.2f);
    node->get_parameter("~body_min_y", body_min_y,  -0.8f);
    node->get_parameter("~body_min_z", body_min_z,  -0.3f);
    node->get_parameter("~obstacle_scale", obstacle_scale,  1.0f);
    roi_max_point = Eigen::Vector4f(roi_max_x, roi_max_y, roi_max_z, 1);
    roi_min_point = Eigen::Vector4f(roi_min_x, roi_min_y, roi_min_z, 1);
    body_max_point = Eigen::Vector4f(body_max_x, body_max_y, body_max_z, 1);
    body_min_point = Eigen::Vector4f(body_min_x, body_min_y, body_min_z, 1);


    // Create publishers and subscribers
    auto cloud_sub = node->create_subscription<avt_341::msg::PointCloud2>(pc_topic, 1, callback_cloud);
    cloud_pub = node->create_publisher<avt_341::msg::PointCloud2>("avt_341/normals_cloud", 1);

    node->spin();
    
    return 0;
}
