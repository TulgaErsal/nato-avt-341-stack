
#ifndef AVT_341_ROS_TYPES_H
#define AVT_341_ROS_TYPES_H

#ifdef ROS_1

#include "ackermann_msgs/AckermannDriveStamped.h"

#include <geometry_msgs/Quaternion.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/NavSatFix.h"

#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/AccelStamped.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GridCells.h"

#include "map_msgs/OccupancyGridUpdate.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "tf/LinearMath/Transform.h"

#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Header.h"
#include "std_msgs/Bool.h"

#include "avt_341_msgs/Communication.h"
#include "avt_341_msgs/NavGoal.h"
#include "avt_341_msgs/NavGoalSequence.h"
#include "avt_341_msgs/NavState.h"
#include "avt_341_msgs/FollowerStatus.h"
#include "avt_341_msgs/MissionTaskStatus.h"
#include "avt_341_msgs/MapMarker.h"
#include "avt_341_msgs/MapMarkerList.h"
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
        using AckermannDriveStampedPtr = ackermann_msgs::AckermannDriveStamped::Ptr;

        using PointCloud = sensor_msgs::PointCloud;
        using PointCloudPtr = const sensor_msgs::PointCloud::ConstPtr &;

        using PointCloud2 = sensor_msgs::PointCloud2;
        using PointCloud2Ptr = const sensor_msgs::PointCloud2::ConstPtr &;
        template<typename T>
        using PointCloud2Iterator = sensor_msgs::PointCloud2Iterator<T>;

        using PointField = sensor_msgs::PointField;
        using PointFieldPtr = const sensor_msgs::PointField::ConstPtr &;

        using JointState = sensor_msgs::JointState;
        using JointStatePtr = const sensor_msgs::JointState::ConstPtr &;

        using Imu = sensor_msgs::Imu;
        using ImuPtr = const sensor_msgs::Imu::ConstPtr &;

        using Image = sensor_msgs::Image;
        using ImagePtr = const sensor_msgs::Image::ConstPtr &;

        using CameraInfo = sensor_msgs::CameraInfo;
        using CameraInfoPtr = const sensor_msgs::CameraInfo::ConstPtr &;

        using TwistStamped = geometry_msgs::TwistStamped;
        using TwistStampedPtr = const geometry_msgs::TwistStamped::ConstPtr &;

        using Imu = sensor_msgs::Imu;
        using ImuPtr = const sensor_msgs::Imu::ConstPtr &;

        using NavSatFix = sensor_msgs::NavSatFix;
        using NavSatFixPtr = const sensor_msgs::NavSatFix::ConstPtr &;

        using Twist = geometry_msgs::Twist;
        using TwistPtr = const geometry_msgs::Twist::ConstPtr &;

        using Point = geometry_msgs::Point;
        using PointPtr = const geometry_msgs::Point::ConstPtr &;

        using Point32 = geometry_msgs::Point32;
        using Point32Ptr = const geometry_msgs::Point32::ConstPtr &;

        using Quaternion = geometry_msgs::Quaternion;
        using QuaternionPtr = const geometry_msgs::Quaternion::ConstPtr &;

        using Point = geometry_msgs::Point;
        using PointPtr = const geometry_msgs::Point::ConstPtr &;

        using PoseStamped = geometry_msgs::PoseStamped;
        using PoseStampedPtr = const geometry_msgs::PoseStamped::ConstPtr &;

        using Pose = geometry_msgs::Pose;
        using PosePtr = const geometry_msgs::Pose::ConstPtr &;

        using PointStamped = geometry_msgs::PointStamped;
        using PointStampedPtr = const geometry_msgs::PointStamped::ConstPtr &;

        using TransformStamped = geometry_msgs::TransformStamped;
        using TransformStampedPtr = const geometry_msgs::TransformStampedConstPtr &;

        using Transform = geometry_msgs::Transform;
        using TransformPtr = const geometry_msgs::TransformConstPtr &;

        using Vector3 = geometry_msgs::Vector3;
        using Vector3Ptr = const geometry_msgs::Vector3ConstPtr &;

        using AccelStamped = geometry_msgs::AccelStamped;
        using AccelStampedPtr = const geometry_msgs::AccelStampedConstPtr &;

        using OccupancyGrid = nav_msgs::OccupancyGrid;
        using OccupancyGridPtr = const nav_msgs::OccupancyGrid::ConstPtr &;
        using OccupancyGridSharedPtr = nav_msgs::OccupancyGrid::Ptr;

        using OccupancyGridUpdate = map_msgs::OccupancyGridUpdate;
        using OccupancyGridUpdatePtr = const map_msgs::OccupancyGridUpdate::ConstPtr &;

        using Path = nav_msgs::Path;
        using PathPtr = const nav_msgs::Path::ConstPtr &;

        using Odometry = nav_msgs::Odometry;
        using OdometryPtr = const nav_msgs::Odometry::ConstPtr &;

        using GridCells = nav_msgs::GridCells;
        using GridCellsPtr = const nav_msgs::GridCells::ConstPtr &;

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

        using Header = std_msgs::Header;
        using HeaderPtr = const std_msgs::Header::ConstPtr &;

        using Bool = std_msgs::Bool;
        using BoolPtr = const std_msgs::Bool::ConstPtr &;

        using FollowerStatus = avt_341_msgs::FollowerStatus;
        using FollowerStatusPtr = const avt_341_msgs::FollowerStatus::ConstPtr &;
        
        using Communication = avt_341_msgs::Communication;
        using CommunicationPtr = const avt_341_msgs::Communication::ConstPtr &;

        using NavGoal = avt_341_msgs::NavGoal;
        using NavGoalPtr = const avt_341_msgs::NavGoal::ConstPtr &;

        using NavGoalSequence = avt_341_msgs::NavGoalSequence;
        using NavGoalSequencePtr = const avt_341_msgs::NavGoalSequence::ConstPtr &;

        using NavState = avt_341_msgs::NavState;
        using NavStatePtr = const avt_341_msgs::NavState::ConstPtr &;

        using MissionTaskStatus = avt_341_msgs::MissionTaskStatus;
        using MissionTaskStatusPtr = const avt_341_msgs::MissionTaskStatus::ConstPtr &;

        using MapMarker = avt_341_msgs::MapMarker;
        using MapMarkerPtr = const avt_341_msgs::MapMarker::ConstPtr &;

        using MapMarkerList = avt_341_msgs::MapMarkerList;
        using MapMarkerListPtr = const avt_341_msgs::MapMarkerList::ConstPtr &;

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
        using Duration = ros::Duration;
        using DurationMsg = ros::Duration;
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
#include <rclcpp/duration.hpp>
#include "builtin_interfaces/msg/duration.hpp"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/grid_cells.hpp"

#include "map_msgs/msg/occupancy_grid_update.hpp"

#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/bool.hpp"

#include "avt_341_msgs/msg/communication.hpp"
#include "avt_341_msgs/msg/compute_time.hpp"
#include "avt_341_msgs/msg/compute_time_array.hpp"
#include "avt_341_msgs/msg/nav_goal.hpp"
#include "avt_341_msgs/msg/nav_goal_sequence.hpp"
#include "avt_341_msgs/msg/nav_state.hpp"
#include "avt_341_msgs/msg/follower_status.hpp"
#include "avt_341_msgs/msg/mission_task_status.hpp"
#include "avt_341_msgs/msg/mission_module_status.hpp"
#include "avt_341_msgs/msg/map_marker.hpp"
#include "avt_341_msgs/msg/map_marker_list.hpp"
#include "avt_341_msgs/msg/occupied_cell.hpp"
#include "avt_341_msgs/msg/occupied_cells.hpp"
#include "avt_341_msgs/msg/obstacles.hpp"
#include "avt_341_msgs/msg/sinkage.hpp"
#include "avt_341_msgs/msg/dwa_info.hpp"
#include "avt_341_msgs/msg/dwa_objective.hpp"
#include "avt_341_msgs/msg/dwa_trajectory.hpp"

namespace avt_341 {
  namespace msg {
    using AckermannDriveStamped = ackermann_msgs::msg::AckermannDriveStamped;
    using AckermannDriveStampedPtr = ackermann_msgs::msg::AckermannDriveStamped::SharedPtr;

    using PointCloud = sensor_msgs::msg::PointCloud;
    using PointCloudPtr = sensor_msgs::msg::PointCloud::SharedPtr;

    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using PointCloud2Ptr = const sensor_msgs::msg::PointCloud2::SharedPtr;
    template<typename T>
    using PointCloud2Iterator = sensor_msgs::PointCloud2Iterator<T>;

    using PointField = sensor_msgs::msg::PointField;
    using PointFieldPtr = sensor_msgs::msg::PointField::SharedPtr;

    using JointState = sensor_msgs::msg::JointState;
    using JointStatePtr = sensor_msgs::msg::JointState::SharedPtr;

    using Image = sensor_msgs::msg::Image;
    using ImagePtr = sensor_msgs::msg::Image::ConstSharedPtr;

    using CameraInfo = sensor_msgs::msg::CameraInfo;
    using CameraInfoPtr = sensor_msgs::msg::CameraInfo::ConstSharedPtr;

    using Imu = sensor_msgs::msg::Imu;
    using ImuPtr = sensor_msgs::msg::Imu::ConstSharedPtr;

    using TwistStamped = geometry_msgs::msg::TwistStamped;
    using TwistStampedPtr = const geometry_msgs::msg::TwistStamped::SharedPtr &;

    using Twist = geometry_msgs::msg::Twist;
    using TwistPtr = const geometry_msgs::msg::Twist::SharedPtr;

    using Point = geometry_msgs::msg::Point;
    using PointPtr = geometry_msgs::msg::Point::SharedPtr;

    using Point32 = geometry_msgs::msg::Point32;
    using Point32Ptr = geometry_msgs::msg::Point32::SharedPtr;

    using Quaternion = geometry_msgs::msg::Quaternion;
    using QuaternionPtr = geometry_msgs::msg::Quaternion::SharedPtr;

    using Point = geometry_msgs::msg::Point;
    using PointPtr = geometry_msgs::msg::Point::SharedPtr;

    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using PoseStampedPtr = geometry_msgs::msg::PoseStamped::SharedPtr;

    using Pose = geometry_msgs::msg::Pose;
    using PosePtr = geometry_msgs::msg::Pose::SharedPtr;

    using PointStamped = geometry_msgs::msg::PointStamped;
    using PointStampedPtr = const geometry_msgs::msg::PointStamped::SharedPtr;

    using TransformStamped = geometry_msgs::msg::TransformStamped;
    using TransformStampedPtr = geometry_msgs::msg::TransformStamped::SharedPtr;

    using Transform = geometry_msgs::msg::Transform;
    using TransformPtr = geometry_msgs::msg::Transform::SharedPtr;

    using Vector3 = geometry_msgs::msg::Vector3;
    using Vector3Ptr = geometry_msgs::msg::Vector3::SharedPtr;

    using AccelStamped = geometry_msgs::msg::AccelStamped;
    using AccelStampedPtr = geometry_msgs::msg::AccelStamped::SharedPtr;

    using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
    using OccupancyGridPtr = nav_msgs::msg::OccupancyGrid::SharedPtr;
    using OccupancyGridSharedPtr = nav_msgs::msg::OccupancyGrid::SharedPtr;

    using OccupancyGridUpdate = map_msgs::msg::OccupancyGridUpdate;
    using OccupancyGridUpdatePtr = map_msgs::msg::OccupancyGridUpdate::SharedPtr;

    using Path = nav_msgs::msg::Path;
    using PathPtr = nav_msgs::msg::Path::SharedPtr;

    using Odometry = nav_msgs::msg::Odometry;
    using OdometryPtr = nav_msgs::msg::Odometry::SharedPtr;

    using GridCells = nav_msgs::msg::GridCells;
    using GridCellsPtr = nav_msgs::msg::GridCells::SharedPtr;

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

    using ComputeTime = avt_341_msgs::msg::ComputeTime;
    using ComputeTimePtr = avt_341_msgs::msg::ComputeTime::SharedPtr;

    using ComputeTimeArray = avt_341_msgs::msg::ComputeTimeArray;
    using ComputeTimeArrayPtr = avt_341_msgs::msg::ComputeTimeArray::SharedPtr;

    using NavGoal = avt_341_msgs::msg::NavGoal;
    using NavGoalPtr = avt_341_msgs::msg::NavGoal::SharedPtr;

    using NavGoalSequence = avt_341_msgs::msg::NavGoalSequence;
    using NavGoalSequencePtr = avt_341_msgs::msg::NavGoalSequence::SharedPtr;

    using NavState = avt_341_msgs::msg::NavState;
    using NavStatePtr = avt_341_msgs::msg::NavState::SharedPtr;

    using MissionTaskStatus = avt_341_msgs::msg::MissionTaskStatus;
    using MissionTaskStatusPtr = avt_341_msgs::msg::MissionTaskStatus::SharedPtr;

    using MissionModuleStatus = avt_341_msgs::msg::MissionModuleStatus;
    using MissionModuleStatusPtr = avt_341_msgs::msg::MissionModuleStatus::SharedPtr;

    using MapMarker = avt_341_msgs::msg::MapMarker;
    using MapMarkerPtr = avt_341_msgs::msg::MapMarker::SharedPtr;

    using MapMarkerList = avt_341_msgs::msg::MapMarkerList;
    using MapMarkerListPtr = avt_341_msgs::msg::MapMarkerList::SharedPtr;

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
    
    using Header = std_msgs::msg::Header;
    using HeaderPtr = std_msgs::msg::Header::SharedPtr;

    using String = std_msgs::msg::String;
    using StringPtr = const std_msgs::msg::String::SharedPtr;

    using Bool = std_msgs::msg::Bool;
    using BoolPtr = const std_msgs::msg::Bool::SharedPtr;

    using DwaInfo = avt_341_msgs::msg::DwaInfo;
    using DwaTrajectory = avt_341_msgs::msg::DwaTrajectory;
    using DwaObjective = avt_341_msgs::msg::DwaObjective;

    using Time = rclcpp::Time;
    using Duration = rclcpp::Duration;
    using DurationMsg = builtin_interfaces::msg::Duration;
  }
  namespace msg_tf{
    using Matrix3x3 = tf2::Matrix3x3;
    using Quaternion = tf2::Quaternion;
    using Vector3 = tf2::Vector3;
  }
}

#endif // ROS_1

#endif //AVT_341_ROS_TYPES_H
