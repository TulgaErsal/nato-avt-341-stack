#ifndef _IMU_PREINTEGRATION_LIDAR_ODOMETRY_H_
#define _IMU_PREINTEGRATION_LIDAR_ODOMETRY_H_

#include "avt_341/perception/slam/utility_node_proxy.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

class TransformFusion : public ParamServer
{
public:
    std::mutex mtx;

    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Odometry>> pubImuOdometry = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Path>> pubImuPath = nullptr;

    Eigen::Affine3f lidarOdomAffine;
    Eigen::Affine3f imuOdomAffineFront;
    Eigen::Affine3f imuOdomAffineBack;

    //tf::TransformListener tfListener; TODO
    avt_341::msg_tf::Transform lidar2Baselink;

    bool lidar2Baselink_initialized{false};

    double lidarOdomTime = -1;
    deque<avt_341::msg::Odometry> imuOdomQueue;

    TransformFusion(std::shared_ptr<avt_341::node::NodeProxy>);

    
    Eigen::Affine3f odom2affine(avt_341::msg::Odometry odom);

    void lidarOdometryHandler(avt_341::msg::OdometryPtr odomMsg);
    void broadcastTransformation(avt_341::msg::Odometry odomMsg);
    void imuOdometryHandler(avt_341::msg::OdometryPtr odomMsg);
};

class IMUPreintegration : public ParamServer
{
public:
    std::mutex mtx;

    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Odometry>> pubImuOdometry;

    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;

    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    std::deque<avt_341::msg::Imu> imuQueOpt;
    std::deque<avt_341::msg::Imu> imuQueImu;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::NavState prePrevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;

    int key = 1;

    // T_bl: tramsform points from lidar frame to imu frame
    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    // T_lb: tramsform points from imu frame to lidar frame
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    IMUPreintegration(std::shared_ptr<avt_341::node::NodeProxy>);
    void resetOptimization();
    void resetParams();
    void odometryHandler(avt_341::msg::OdometryPtr odomMsg);
    bool failureDetection(const gtsam::Vector3 &velCur, const gtsam::imuBias::ConstantBias &biasCur);
    void imuHandler(avt_341::msg::ImuPtr imu_raw);
};

#endif
