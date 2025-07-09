
#ifndef AVT_341_ROS_TYPES_H
#define AVT_341_ROS_TYPES_H

#ifdef ROS_1

#include "ackermann_msgs/AckermannDriveStamped.h"

#include <geometry_msgs/Quaternion.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "tf/LinearMath/Transform.h"

#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64MultiArray.h"

#include "avt_341_msgs/Communication.h"
#include "avt_341_msgs/FollowerStatus.h"
#include "avt_341_msgs/OccupiedCell.h"
#include "avt_341_msgs/OccupiedCells.h"
#include "avt_341_msgs/Obstacles.h"
#include "avt_341_msgs/Sinkage.h"
#include "avt_341_msgs/LiorfCloudInfo.h"
#include "avt_341_msgs/LiorfSaveMap.h"
#include "avt_341_msgs/DwaInfo.h"
#include "avt_341_msgs/DwaObjective.h"
#include "avt_341_msgs/DwaTrajectory.h"

namespace avt_341 {
    namespace msg {
        using AckermannDriveStamped = ackermann_msgs::AckermannDriveStamped;
        using AckermannDriveStampedPtr = boost::shared_ptr<ackermann_msgs::AckermannDriveStamped>;

        using PointCloud = sensor_msgs::PointCloud;
        using PointCloudPtr = const sensor_msgs::PointCloud::ConstPtr &;

        using PointCloud2 = sensor_msgs::PointCloud2;
        using PointCloud2Ptr = const sensor_msgs::PointCloud2::ConstPtr &;

        using PointField = sensor_msgs::PointField;
        using PointFieldPtr = const sensor_msgs::PointField::ConstPtr &;

        using JointState = sensor_msgs::JointState;
        using JointStatePtr = const sensor_msgs::JointState::ConstPtr &;

        using Image = sensor_msgs::Image;
        using ImagePtr = const sensor_msgs::Image::ConstPtr &;

        using Imu = sensor_msgs::Imu;
        using ImuPtr = const sensor_msgs::Imu::ConstPtr &;

        using NavSatFix = sensor_msgs::NavSatFix;
        using NavSatFixPtr = const sensor_msgs::NavSatFix::ConstPtr &;

        using Twist = geometry_msgs::Twist;
        using TwistPtr = const geometry_msgs::Twist::ConstPtr &;

        using Point32 = geometry_msgs::Point32;
        using Point32Ptr = const geometry_msgs::Point32::ConstPtr &;

        using Quaternion = geometry_msgs::Quaternion;
        using QuaternionPtr = const geometry_msgs::Quaternion::ConstPtr &;

        using Point = geometry_msgs::Point;
        using PointPtr = const geometry_msgs::Point::ConstPtr &;

        using PoseStamped = geometry_msgs::PoseStamped;
        using PoseStampedPtr = const geometry_msgs::PoseStamped::ConstPtr &;

        using Pose = geometry_msgs::Pose;

        using PointStamped = geometry_msgs::PointStamped;
        using PointStampedPtr = const geometry_msgs::PointStamped::ConstPtr &;

        using TransformStamped = geometry_msgs::TransformStamped;
        using TransformStampedPtr = const geometry_msgs::TransformStampedConstPtr &;

        using OccupancyGrid = nav_msgs::OccupancyGrid;
        using OccupancyGridPtr = const nav_msgs::OccupancyGrid::ConstPtr &;

        using Path = nav_msgs::Path;
        using PathPtr = const nav_msgs::Path::ConstPtr &;

        using Odometry = nav_msgs::Odometry;
        using OdometryPtr = const nav_msgs::Odometry::ConstPtr &;

        using Marker = visualization_msgs::Marker;
        using MarkerPtr = const visualization_msgs::Marker::ConstPtr &;

        using MarkerArray = visualization_msgs::MarkerArray;
        using MarkerArrayPtr = const visualization_msgs::MarkerArray::ConstPtr &;

        using Float64 = std_msgs::Float64;
        using Float64Ptr = const std_msgs::Float64::ConstPtr &;

        using Float64MultiArray = std_msgs::Float64MultiArray;
        using Float64MultiArrayPtr = const std_msgs::Float64MultiArray::ConstPtr &;

        using MultiArrayDimension = std_msgs::MultiArrayDimension;
        using MultiArrayDimensionPtr = const std_msgs::MultiArrayDimension::ConstPtr &;

        using Int32 = std_msgs::Int32;
        using Int32Ptr = const std_msgs::Int32::ConstPtr &;

        using Header = std_msgs::Header;
        using HeaderPtr = const std_msgs::Header::ConstPtr &;

        using String = std_msgs::String;
        using StringPtr = const std_msgs::String::ConstPtr &;

        using FollowerStatus = avt_341_msgs::FollowerStatus;
        using FollowerStatusPtr = const avt_341_msgs::FollowerStatus::ConstPtr &;
        
        using Communication = avt_341_msgs::Communication;
        using CommunicationPtr = const avt_341_msgs::Communication::ConstPtr &;

        using OccupiedCell = avt_341_msgs::OccupiedCell;
        using OccupiedCellPtr = const avt_341_msgs::OccupiedCell::ConstPtr &;

        using OccupiedCells = avt_341_msgs::OccupiedCells;
        using OccupiedCellsPtr = const avt_341_msgs::OccupiedCells::ConstPtr &;

        using Obstacles = avt_341_msgs::Obstacles;
        using ObstaclesPtr = const avt_341_msgs::Obstacles::ConstPtr &;

        using Sinkage = avt_341_msgs::Sinkage;
        using SinkagePtr = const avt_341_msgs::Sinkage::ConstPtr &;

        using LiorfCloudInfo = avt_341_msgs::LiorfCloudInfo;
        using LiorfCloudInfoPtr = const avt_341_msgs::LiorfCloudInfo::ConstPtr &;

        using LiorfSaveMapRequest = avt_341_msgs::LiorfSaveMapRequest;
        using LiorfSaveMapResponse = avt_341_msgs::LiorfSaveMapResponse;

        using DwaInfo = avt_341_msgs::DwaInfo;
        using DwaTrajectory = avt_341_msgs::DwaTrajectory;
        using DwaObjective = avt_341_msgs::DwaObjective;


        using Time = ros::Time;
    }
    namespace msg_tf{
        using Matrix3x3 = tf::Matrix3x3;
        using Quaternion = tf::Quaternion;
        using Vector3 = tf::Vector3;
    }
}

#else

#include <cstring>
#include <rclcpp/time.hpp>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

#include "avt_341_msgs/msg/communication.hpp"
#include "avt_341_msgs/msg/follower_status.hpp"
#include "avt_341_msgs/msg/occupied_cell.hpp"
#include "avt_341_msgs/msg/occupied_cells.hpp"
#include "avt_341_msgs/msg/obstacles.hpp"
#include "avt_341_msgs/msg/sinkage.hpp"
#include "avt_341_msgs/msg/DwaInfo.hpp"
#include "avt_341_msgs/msg/DwaObjective.hpp"
#include "avt_341_msgs/msg/DwaTrajectory.hpp"

namespace avt_341 {
  namespace msg {
    using AckermannDriveStamped = ackermann_msgs::msg::AckermannDriveStamped;
    using AckermannDriveStampedPtr = ackermann_msgs::msg::AckermannDriveStamped::SharedPtr;

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

    using Imu = sensor_msgs::msg::Imu;
    using ImuPtr = sensor_msgs::msg::Imu::ConstSharedPtr;

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

    using Pose = geometry_msgs::msg::Pose;

    using PointStamped = geometry_msgs::msg::PointStamped;
    using PointStampedPtr = const geometry_msgs::msg::PointStamped::SharedPtr;

    using TransformStamped = geometry_msgs::msg::TransformStamped;
    using TransformStampedPtr = geometry_msgs::msg::TransformStamped::SharedPtr;

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

    using FollowerStatus = avt_341_msgs::msg::FollowerStatus;
    using FollowerStatusPtr = avt_341_msgs::msg::FollowerStatus::SharedPtr;

    using Communication = avt_341_msgs::msg::Communication;
    using CommunicationPtr = avt_341_msgs::msg::Communication::SharedPtr;

    using OccupiedCell = avt_341_msgs::msg::OccupiedCell;
    using OccupiedCellPtr = avt_341_msgs::msg::OccupiedCell::SharedPtr;

    using OccupiedCells = avt_341_msgs::msg::OccupiedCells;
    using OccupiedCellsPtr = avt_341_msgs::msg::OccupiedCells::SharedPtr;

    using Obstacles = avt_341_msgs::msg::Obstacles;
    using ObstaclesPtr = avt_341_msgs::msg::Obstacles::SharedPtr;

    using Sinkage = avt_341_msgs::msg::Sinkage;
    using SinkagePtr = avt_341_msgs::msg::Sinkage::SharedPtr;

    using Float64MultiArray = std_msgs::msg::Float64MultiArray;
    using Float64MultiArrayPtr = std_msgs::msg::Float64MultiArray::SharedPtr;
    using MultiArrayDimension = std_msgs::msg::MultiArrayDimension;
    
    using String = std_msgs::msg::String;
    using StringPtr = const std_msgs::msg::String::SharedPtr;

    using DwaInfo = avt_341_msgs::msg::DwaInfo;
    using DwaTrajectory = avt_341_msgs::msg::DwaTrajectory;
    using DwaObjective = avt_341_msgs::msg::DwaObjective;

    using Time = rclcpp::Time;
  }
  namespace msg_tf{
    using Matrix3x3 = tf2::Matrix3x3;
    using Quaternion = tf2::Quaternion;
    using Vector3 = tf2::Vector3;
  }
}

#endif // ROS_1

#endif //AVT_341_ROS_TYPES_H
