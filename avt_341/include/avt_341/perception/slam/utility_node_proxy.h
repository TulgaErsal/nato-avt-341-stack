#pragma once
#ifndef _UTILITY_NODE_PROXY_LIDAR_ODOMETRY_H_
#define _UTILITY_NODE_PROXY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE
// <!-- liorf_yjz_lucky_boy -->

#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include <common_lib.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
// #include <opencv/cv.h>

//#include <tf/LinearMath/Quaternion.h>
//#include <tf/transform_listener.h>
//#include <tf/transform_datatypes.h>
//#include <tf/transform_broadcaster.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

typedef pcl::PointXYZI PointType;

// <!-- liorf_localization_yjz_lucky_boy -->
std::shared_ptr<CommonLib::common_lib> common_lib_;

enum class SensorType
{
    VELODYNE,
    OUSTER,
    LIVOX,
    ROBOSENSE,
    MULRAN
};

class ParamServer
{
public:
    std::shared_ptr<avt_341::node::NodeProxy> node_proxy = nullptr;

    std::string robot_id;

    bool debug;
    bool useIMU;

    // Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;
    string additionalOdomTopic;

    // Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Lidar Sensor Configuration
    SensorType sensor;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    int point_filter_num;
    float lidarMinRange;
    float lidarMaxRange;
    int additionalUncertainty;

    // IMU
    int imuType;
    float imuRate;
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    vector<double> odometerTransV;
    vector<double> odometerRotV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;
    Eigen::Vector3d odometerTrans;
    Eigen::Quaterniond odometerRot;

    // voxel filter paprams
    float mappingSurfLeafSize;
    float surroundingKeyframeMapLeafSize;
    float loopClosureICPSurfLeafSize;

    float z_tollerance;
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold;
    float surroundingkeyframeAddingAngleThreshold;
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;

    // Loop closure
    bool loopClosureEnableFlag;
    float loopClosureFrequency;
    int surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    // robustness parameters
    std::string defaultOdomSource; // "lidar" or "additional"
    bool useBestOdom;              // if to switch from the default odometry or not
    float thRotationSwitch;        // which roatational difference with IMU would be an indication to switch. possibl difference from 0 to sqrt(2)
    float thTranslationSwitch;     // which translational difference with IMU would be an indication to switch. possibl difference from 0 to sqrt(2)
    double rosbagStart;            //
    int maxIMUgraphLen;            //
    bool useOnlyIsDegenerate;

    bool adjustAdditionalOdomScale;
    int scaleQueueSize;
    float defaultAdditionalScale;

    bool useIsDegenerate;
    bool calculateInitRollPitch{true};
    int initRollPitchWindow{50};
    bool isImuNed{false};

    Eigen::Quaterniond ned_q_enu{};

    ParamServer(std::shared_ptr<avt_341::node::NodeProxy> _node_proxy)
    {
        node_proxy = _node_proxy;

        node_proxy->get_parameter<std::string>("/robot_id", robot_id, "roboat");

        node_proxy->get_parameter<bool>("liorf/debug", debug, false);
        node_proxy->get_parameter<bool>("liorf/useIMU", useIMU, true);
        
        node_proxy->get_parameter<std::string>("liorf/pointCloudTopic", pointCloudTopic, "points_raw");
        node_proxy->get_parameter<std::string>("liorf/imuTopic", imuTopic, "imu_correct");
        node_proxy->get_parameter<std::string>("liorf/odomTopic", odomTopic, "odometry/imu");
        node_proxy->get_parameter<std::string>("liorf/gpsTopic", gpsTopic, "odometry/gps");
        node_proxy->get_parameter<std::string>("liorf/additionalOdomTopic", additionalOdomTopic, "/odom_wheels");

        node_proxy->get_parameter<std::string>("liorf/lidarFrame", lidarFrame, "base_link");
        node_proxy->get_parameter<std::string>("liorf/baselinkFrame", baselinkFrame, "base_link");
        node_proxy->get_parameter<std::string>("liorf/odometryFrame", odometryFrame, "odom");
        node_proxy->get_parameter<std::string>("liorf/mapFrame", mapFrame, "map");

        node_proxy->get_parameter<bool>("liorf/useImuHeadingInitialization", useImuHeadingInitialization, false);
        node_proxy->get_parameter<bool>("liorf/useGpsElevation", useGpsElevation, false);
        node_proxy->get_parameter<float>("liorf/gpsCovThreshold", gpsCovThreshold, 2.0);
        node_proxy->get_parameter<float>("liorf/poseCovThreshold", poseCovThreshold, 25.0);
        node_proxy->get_parameter<bool>("liorf/calculateInitRollPitch", calculateInitRollPitch, true);
        node_proxy->get_parameter<int>("liorf/initRollPitchWindow", initRollPitchWindow, 50);

        node_proxy->get_parameter<bool>("liorf/savePCD", savePCD, false);
        node_proxy->get_parameter<std::string>("liorf/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

        std::string sensorStr;
        node_proxy->get_parameter<std::string>("liorf/sensor", sensorStr, "");
        if (sensorStr == "velodyne")
        {
            sensor = SensorType::VELODYNE;
        }
        else if (sensorStr == "ouster")
        {
            sensor = SensorType::OUSTER;
        }
        else if (sensorStr == "livox")
        {
            sensor = SensorType::LIVOX;
        }
        else if (sensorStr == "robosense")
        {
            sensor = SensorType::ROBOSENSE;
        }
        else if (sensorStr == "mulran")
        {
            sensor = SensorType::MULRAN;
        }
        else
        {
            //ROS_ERROR_STREAM("Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox' or 'robosense' or 'mulran'): " << sensorStr);
            node_proxy->log_error("Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox' or 'robosense' or 'mulran'): TODO stream"); //TODO
            node_proxy->shutdown();
        }

        node_proxy->get_parameter<int>("liorf/N_SCAN", N_SCAN, 16);
        node_proxy->get_parameter<int>("liorf/Horizon_SCAN", Horizon_SCAN, 1800);
        node_proxy->get_parameter<int>("liorf/downsampleRate", downsampleRate, 1);
        node_proxy->get_parameter<int>("liorf/point_filter_num", point_filter_num, 3);
        node_proxy->get_parameter<float>("liorf/lidarMinRange", lidarMinRange, 1.0);
        node_proxy->get_parameter<float>("liorf/lidarMaxRange", lidarMaxRange, 1000.0);
        node_proxy->get_parameter<int>("liorf/additionalUncertainty", additionalUncertainty, 10);

        node_proxy->get_parameter<int>("liorf/imuType", imuType, 0);
        node_proxy->get_parameter<float>("liorf/imuRate", imuRate, 500.0);
        node_proxy->get_parameter<float>("liorf/imuAccNoise", imuAccNoise, 0.01);
        node_proxy->get_parameter<float>("liorf/imuGyrNoise", imuGyrNoise, 0.001);
        node_proxy->get_parameter<float>("liorf/imuAccBiasN", imuAccBiasN, 0.0002);
        node_proxy->get_parameter<float>("liorf/imuGyrBiasN", imuGyrBiasN, 0.00003);
        node_proxy->get_parameter<float>("liorf/imuGravity", imuGravity, 9.80511);
        node_proxy->get_parameter<float>("liorf/imuRPYWeight", imuRPYWeight, 0.01);
        node_proxy->get_parameter<vector<double>>("liorf/extrinsicRot", extRotV, vector<double>());
        node_proxy->get_parameter<vector<double>>("liorf/extrinsicRPY", extRPYV, vector<double>());
        node_proxy->get_parameter<vector<double>>("liorf/extrinsicTrans", extTransV, vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY).inverse();

        node_proxy->get_parameter<vector<double>>("liorf/odometerTrans", odometerTransV, vector<double>());
        node_proxy->get_parameter<vector<double>>("liorf/odometerRot", odometerRotV, vector<double>());
        if (odometerTransV.size() == 3 && odometerRotV.size() == 4)
        {
            odometerTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(odometerTransV.data(), 3, 1);
            odometerRot = Eigen::Quaterniond(odometerRotV.data());
        } else {
            odometerTrans = Eigen::Vector3d::Zero();
            odometerRot = Eigen::Quaterniond::Identity();
        }
        

        node_proxy->get_parameter<float>("liorf/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);
        node_proxy->get_parameter<float>("liorf/surroundingKeyframeMapLeafSize", surroundingKeyframeMapLeafSize, 0.2);

        node_proxy->get_parameter<float>("liorf/z_tollerance", z_tollerance, FLT_MAX);
        node_proxy->get_parameter<float>("liorf/rotation_tollerance", rotation_tollerance, FLT_MAX);

        node_proxy->get_parameter<int>("liorf/numberOfCores", numberOfCores, 2);
        node_proxy->get_parameter<double>("liorf/mappingProcessInterval", mappingProcessInterval, 0.15);

        node_proxy->get_parameter<float>("liorf/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        node_proxy->get_parameter<float>("liorf/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        node_proxy->get_parameter<float>("liorf/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        node_proxy->get_parameter<float>("liorf/loopClosureICPSurfLeafSize", loopClosureICPSurfLeafSize, 0.3);
        node_proxy->get_parameter<float>("liorf/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        node_proxy->get_parameter<bool>("liorf/loopClosureEnableFlag", loopClosureEnableFlag, false);
        node_proxy->get_parameter<float>("liorf/loopClosureFrequency", loopClosureFrequency, 1.0);
        node_proxy->get_parameter<int>("liorf/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        node_proxy->get_parameter<float>("liorf/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        node_proxy->get_parameter<float>("liorf/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        node_proxy->get_parameter<int>("liorf/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        node_proxy->get_parameter<float>("liorf/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

        node_proxy->get_parameter<float>("liorf/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        node_proxy->get_parameter<float>("liorf/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        node_proxy->get_parameter<float>("liorf/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        node_proxy->get_parameter<std::string>("liorf/defaultOdomSource", defaultOdomSource, "lidar");
        node_proxy->get_parameter<bool>("liorf/useBestOdom", useBestOdom, false);
        node_proxy->get_parameter<float>("liorf/thRotationSwitch", thRotationSwitch, 0.1);
        node_proxy->get_parameter<float>("liorf/thTranslationSwitch", thTranslationSwitch, 0.05);
        node_proxy->get_parameter<double>("liorf/rosbagStart", rosbagStart, double(1678964096.007040));
        node_proxy->get_parameter<int>("liorf/maxIMUgraphLen", maxIMUgraphLen, 100);
        node_proxy->get_parameter<bool>("liorf/useOnlyIsDegenerate", useOnlyIsDegenerate, false);
        node_proxy->get_parameter<bool>("liorf/adjustAdditionalOdomScale", adjustAdditionalOdomScale, false);
        node_proxy->get_parameter<int>("liorf/scaleQueueSize", scaleQueueSize, 500);
        node_proxy->get_parameter<bool>("liorf/useIsDegenerate", useIsDegenerate, true);
        node_proxy->get_parameter<float>("liorf/defaultAdditionalScale", defaultAdditionalScale, 1.0);
        node_proxy->get_parameter<bool>("liorf/isImuNed", isImuNed, false);

        Eigen::Matrix3d R_ned2enu;
        R_ned2enu << 0, 1, 0,
                    1, 0, 0,
                    0, 0, -1;
        ned_q_enu = Eigen::Quaterniond(R_ned2enu);

        usleep(100);
    }

    avt_341::msg::Imu imuConverter(const avt_341::msg::Imu &imu_in)
    {
        avt_341::msg::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();

        if (imuType)
        {
            // rotate roll pitch yaw
            Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
            Eigen::Quaterniond q_final = q_from * extQRPY;
            if (isImuNed)
            {
                q_final = ned_q_enu * q_final;
            }
            imu_out.orientation.x = q_final.x();
            imu_out.orientation.y = q_final.y();
            imu_out.orientation.z = q_final.z();
            imu_out.orientation.w = q_final.w();

            if (sqrt(q_final.x() * q_final.x() + q_final.y() * q_final.y() + q_final.z() * q_final.z() + q_final.w() * q_final.w()) < 0.1)
            {
                node_proxy->log_error("Invalid quaternion, please use a 9-axis IMU!");
                node_proxy->shutdown();
            }
        }

        return imu_out;
    }
};

template <typename T>
//avt_341::msg::PointCloud2 publishCloud(const ros::Publisher &thisPub, const T &thisCloud, avt_341::msg::Time thisStamp, std::string thisFrame)
avt_341::msg::PointCloud2 publishCloud(std::shared_ptr<avt_341::node::Publisher<avt_341::msg::PointCloud2>> thisPub, const T &thisCloud, avt_341::msg::Time thisStamp, std::string thisFrame)
{
    avt_341::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    //if (thisPub.getNumSubscribers() != 0) TODO
    if (true)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template <typename T>
double ROS_TIME(T msg)
{
    return avt_341::node::seconds_from_header(msg->header);
}

template <typename T>
void imuAngular2rosAngular(avt_341::msg::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

template <typename T>
void imuAccel2rosAccel(avt_341::msg::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}

avt_341::msg_tf::Quaternion to_tf_quaterion(avt_341::msg::Quaternion& q_msg)
{
  avt_341::msg_tf::Quaternion q_tf(
      q_msg.x,
      q_msg.y,
      q_msg.z,
      q_msg.w
      );
  return q_tf;
}

avt_341::msg::Quaternion from_tf_quaternion(avt_341::msg_tf::Quaternion& q_tf)
{
  avt_341::msg::Quaternion q_msg;
  q_msg.x = q_tf.getX();
  q_msg.y = q_tf.getY();
  q_msg.z = q_tf.getZ();
  q_msg.w = q_tf.getW();
  return q_msg;
}

template <typename T>
void imuRPY2rosRPY(avt_341::msg::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    avt_341::msg_tf::Quaternion orientation = to_tf_quaterion(thisImuMsg->orientation);
    avt_341::msg_tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

avt_341::msg_tf::Transform pose_to_transform(avt_341::msg::Pose pose)
{
  avt_341::msg_tf::Quaternion q(
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w
      );
  avt_341::msg_tf::Vector3 t(
      pose.position.x,
      pose.position.y,
      pose.position.z
      );
  avt_341::msg_tf::Transform T(q,t);
  return T;
}

avt_341::msg_tf::Transform to_tf_transform(avt_341::msg::TransformStamped& tfs)
{
  avt_341::msg_tf::Quaternion q(
      tfs.transform.rotation.x,
      tfs.transform.rotation.y,
      tfs.transform.rotation.z,
      tfs.transform.rotation.w
      );
  avt_341::msg_tf::Vector3 t(
      tfs.transform.translation.x,
      tfs.transform.translation.y,
      tfs.transform.translation.z
      );
  avt_341::msg_tf::Transform T(q,t);
  return T;
}


avt_341::msg::TransformStamped from_tf_transform(avt_341::msg_tf::Transform& T, avt_341::msg::Time& ts, std::string frame_id, std::string child_frame_id)
{
  avt_341::msg::TransformStamped tfs;

  tfs.header.stamp = ts;
  tfs.header.frame_id = frame_id;
  tfs.child_frame_id = child_frame_id;

  tfs.transform.translation.x = T.getOrigin().getX();
  tfs.transform.translation.y = T.getOrigin().getY();
  tfs.transform.translation.z = T.getOrigin().getZ();

  tfs.transform.rotation.x = T.getRotation().getX();
  tfs.transform.rotation.y = T.getRotation().getY();
  tfs.transform.rotation.z = T.getRotation().getZ();
  tfs.transform.rotation.w = T.getRotation().getW();
  
  return tfs;
}

#endif
