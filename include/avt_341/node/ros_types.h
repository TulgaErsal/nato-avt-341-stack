//
// Created by Stefan on 2021-07-28.
//

#ifndef AVT_341_ROS_TYPES_H
#define AVT_341_ROS_TYPES_H

#ifdef ROS_1

#include <geometry_msgs/Quaternion.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/point_cloud_conversion.h"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/Image.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

namespace avt_341 {
  namespace msg {
    using PointCloud = sensor_msgs::msg::PointCloud;
    using PointCloudPtr = sensor_msgs::msg::PointCloud::SharedPtr;

    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using PointCloud2Ptr = sensor_msgs::msg::PointCloud2::SharedPtr;

    using PointField = sensor_msgs::msg::PointField;
    using PointFieldPtr = sensor_msgs::msg::PointField::SharedPtr;

    using JointState = sensor_msgs::msg::JointState;
    using JointStatePtr = sensor_msgs::msg::JointState::SharedPtr;

    using Image = sensor_msgs::msg::Image;
    using ImagePtr = sensor_msgs::msg::Image::ConstSharedPtr;

    using Twist = geometry_msgs::msg::Twist;
    using TwistPtr = const geometry_msgs::msg::Twist::SharedPtr;

    using Point32 = geometry_msgs::msg::Point32;
    using Point32Ptr = geometry_msgs::msg::Point32::SharedPtr;

    using Quaternion = geometry_msgs::msg::Quaternion;
    using QuaternionPtr = geometry_msgs::msg::Quaternion::SharedPtr;

    using Point = geometry_msgs::msg::Point;
    using PointPtr = geometry_msgs::msg::Point::SharedPtr;

    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using PoseStampedPtr = geometry_msgs::msg::PoseStamped::SharedPtr;

    using PointStamped = geometry_msgs::msg::PointStamped;
    using PointStampedPtr = const geometry_msgs::msg::PointStamped::SharedPtr;

    using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
    using OccupancyGridPtr = nav_msgs::msg::OccupancyGrid::SharedPtr;

    using Path = nav_msgs::msg::Path;
    using PathPtr = nav_msgs::msg::Path::SharedPtr;

    using Odometry = nav_msgs::msg::Odometry;
    using OdometryPtr = nav_msgs::msg::Odometry::SharedPtr;

    using Marker = visualization_msgs::msg::Marker;
    using MarkerPtr = visualization_msgs::msg::Marker::SharedPtr;

    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using MarkerArrayPtr = const visualization_msgs::msg::MarkerArray::SharedPtr;

    using Float64 = std_msgs::msg::Float64;
    using Float64Ptr = std_msgs::msg::Float64::SharedPtr;

    using Int32 = std_msgs::msg::Int32;
    using Int32Ptr = std_msgs::msg::Int32::SharedPtr;

<<<<<<< HEAD
    using Float64MultiArray = std_msgs::msg::Float64MultiArray;
    using Float64MultiArrayPtr = std_msgs::msg::Float64MultiArray::SharedPtr;
    using MultiArrayDimension = std_msgs::msg::MultiArrayDimension;
  }
  namespace msg_tf{
    using Matrix3x3 = tf2::Matrix3x3;
    using Quaternion = tf2::Quaternion;
    using Vector3 = tf2::Vector3;
  }
=======
        using MultiArrayDimension = std_msgs::MultiArrayDimension;
        using MultiArrayDimensionPtr = const std_msgs::MultiArrayDimension::ConstPtr &;

        using Int32 = std_msgs::Int32;
        using Int32Ptr = const std_msgs::Int32::ConstPtr &;
    }
    namespace msg_tf{
        using Matrix3x3 = tf::Matrix3x3;
        using Quaternion = tf::Quaternion;
        using Vector3 = tf::Vector3;
    }
>>>>>>> main
}

#else

#include <cstring>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

namespace avt_341 {
  namespace msg {
    using PointCloud = sensor_msgs::msg::PointCloud;
    using PointCloudPtr = sensor_msgs::msg::PointCloud::SharedPtr;

    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using PointCloud2Ptr = sensor_msgs::msg::PointCloud2::SharedPtr;

    using PointField = sensor_msgs::msg::PointField;
    using PointFieldPtr = sensor_msgs::msg::PointField::SharedPtr;

    using JointState = sensor_msgs::msg::JointState;
    using JointStatePtr = sensor_msgs::msg::JointState::SharedPtr;

    using Image = sensor_msgs::msg::Image;
    using ImagePtr = sensor_msgs::msg::Image::ConstSharedPtr;

    using Twist = geometry_msgs::msg::Twist;
    using TwistPtr = const geometry_msgs::msg::Twist::SharedPtr;

    using Point32 = geometry_msgs::msg::Point32;
    using Point32Ptr = geometry_msgs::msg::Point32::SharedPtr;

    using Quaternion = geometry_msgs::msg::Quaternion;
    using QuaternionPtr = geometry_msgs::msg::Quaternion::SharedPtr;

    using Point = geometry_msgs::msg::Point;
    using PointPtr = geometry_msgs::msg::Point::SharedPtr;

    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using PoseStampedPtr = geometry_msgs::msg::PoseStamped::SharedPtr;

    using PointStamped = geometry_msgs::msg::PointStamped;
    using PointStampedPtr = const geometry_msgs::msg::PointStamped::SharedPtr;

    using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
    using OccupancyGridPtr = nav_msgs::msg::OccupancyGrid::SharedPtr;

    using Path = nav_msgs::msg::Path;
    using PathPtr = nav_msgs::msg::Path::SharedPtr;

    using Odometry = nav_msgs::msg::Odometry;
    using OdometryPtr = nav_msgs::msg::Odometry::SharedPtr;

    using Marker = visualization_msgs::msg::Marker;
    using MarkerPtr = visualization_msgs::msg::Marker::SharedPtr;

    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using MarkerArrayPtr = const visualization_msgs::msg::MarkerArray::SharedPtr;

    using Float64 = std_msgs::msg::Float64;
    using Float64Ptr = std_msgs::msg::Float64::SharedPtr;

    using Int32 = std_msgs::msg::Int32;
    using Int32Ptr = std_msgs::msg::Int32::SharedPtr;

    using Float64MultiArray = std_msgs::msg::Float64MultiArray;
    using Float64MultiArrayPtr = std_msgs::msg::Float64MultiArray::SharedPtr;
    using MultiArrayDimension = std_msgs::msg::MultiArrayDimension;
  }
  namespace msg_tf{
    using Matrix3x3 = tf2::Matrix3x3;
    using Quaternion = tf2::Quaternion;
    using Vector3 = tf2::Vector3;
  }
}

#endif // ROS_1

#endif //AVT_341_ROS_TYPES_H
