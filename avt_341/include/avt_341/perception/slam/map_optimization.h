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

#include <set>

#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
// #include <message_filters/time_synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>

#include <gtsam/nonlinear/ISAM2.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
//#include <eigen_conversions/eigen_msg.h>
#include <algorithm>

#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace gtsam;

using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

/*
 * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
 */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY; // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

typedef PointXYZIRPYT PointTypePose;

// define class for offsets queue, which would store the window of last n double offsets for roll or pitch
class OffsetQueue
{
public:
    OffsetQueue(int s) : size(s) {}
    void push(double offset)
    {
        if (offsetQueue_.size() >= size)
        {
            offsetQueue_.pop_front();
        }
        offsetQueue_.push_back(offset);
    }
    double getMedian()
    {
        std::vector<double> offsets;
        for (auto &offset : offsetQueue_)
        {
            offsets.push_back(offset);
        }
        std::sort(offsets.begin(), offsets.end());
        return offsets[offsets.size() / 2];
    }

    bool isFull()
    {
        return offsetQueue_.size() >= size;
    }

    void reset()
    {
        offsetQueue_.clear();
    }

private:
    std::deque<double> offsetQueue_;
    int size;
};


class mapOptimization : public ParamServer
{

public:
    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::PointCloud2>> pubKeyPoses = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::PointCloud2>> pubLaserCloudSurround = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Odometry>> pubLaserOdometryGlobal = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Odometry>> pubLaserOdometryIncremental = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Path>> pubPath = nullptr;

    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::PointCloud2>> pubHistoryKeyFrames = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::PointCloud2>> pubIcpKeyFrames = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::MarkerArray>> pubLoopConstraintEdge = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::PointCloud2>> pubAdditionalUsage = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Marker>> pubAdditionalUsageTime = nullptr;
    
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::PointCloud2>> pubRecentKeyFrames = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::PointCloud2>> pubRecentKeyFrame = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::PointCloud2>> pubCloudRegisteredRaw = nullptr;

    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::LiorfCloudInfo>> pubSLAMInfo;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::Odometry>> pubGpsOdom;

    // TODO cleanup obsolete 
    //ros::Subscriber subAditionalOdom;
    //message_filters::Cache<avt_341::msg::Odometry> *cacheAdditionalOdom;
    //ros::ServiceServer srvSaveMap;

    std::deque<avt_341::msg::Odometry> gpsQueue;
    avt_341::msg::LiorfCloudInfo cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointType>::Ptr cloudAdditionalUsage;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;   // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf feature set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterLocalMapSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization

    Eigen::Affine3f transIncre;

    avt_341::msg::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float transformTobeMapped[6];

    std::mutex mtx;
    std::mutex mtxLoopInfo;

    bool isDegenerate = false;
    cv::Mat matP;

    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;
    map<int, int> loopIndexContainer; // from new to old

    set<int> degeneratedKeyframesSet;
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    deque<avt_341::msg::Float64MultiArray> loopInfoVec;

    avt_341::msg::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;
    Eigen::Affine3f lastOdomIncrement;
    Eigen::Affine3f lastImuTransformation;
    bool odomIncrementHappened{false};

    gtsam::Pose3 odometer2Lidar;

    gtsam::Pose3 lastAdditionalOdometry;
    gtsam::Pose3 additionalOdometryIncrement;

    gtsam::Pose3 lastSavedAdditionalOdometry;
    avt_341::msg::Time last_topic2_time;
    std::deque<avt_341::msg::Odometry::ConstPtr> messages_from_topic2;

    //ros::CallbackQueue additional_queue;

    // gtsam::Pose3 Additional2lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));
    // gtsam::Pose3 lidar2Additional = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));
    bool insertedAdditionalOdom = false;
    bool gotAdditionalOdom = false;
    bool gotAdditionalOdomIncrement = false;
    bool additionalOdomUsed = false;
    int corruptedKeyframesCount = 0;
    vector<double> proportions;
    bool additionalOdomArrived = false;

    double findMedian(vector<double> a);

    GeographicLib::LocalCartesian gps_trans_;

    mapOptimization(std::shared_ptr<avt_341::node::NodeProxy>);

    void allocateMemory();

    avt_341::msg::Odometry::ConstPtr interpolate_odometry(avt_341::msg::OdometryPtr msg1,
                                            avt_341::msg::OdometryPtr msg2,
                                            const avt_341::msg::Time &t);

    void laserCloudInfoHandler(avt_341::msg::LiorfCloudInfoPtr msgIn);

    void gpsHandler(avt_341::msg::NavSatFixPtr gpsMsg);

    void pointAssociateToMap(PointType const *const pi, PointType *const po);
    
    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint);

    gtsam::Pose3 trans2gtsamPose(float transformIn[]);

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint);

    Eigen::Affine3f trans2Affine3f(float transformIn[]);

    PointTypePose trans2PointTypePose(float transformIn[]);

    //bool saveMapService(avt_341::msg::LiorfSaveMapRequest &req, avt_341::msg::LiorfSaveMapResponse &res);

    void visualizeGlobalMapThread();

    void publishGlobalMap();

    void additionalOdomHandler(avt_341::msg::OdometryPtr odomMsg);

    void syncAdditionalOdomHandler(avt_341::msg::OdometryPtr odomMsg);

    void loopClosureThread();

    void loopInfoHandler(avt_341::msg::Float64MultiArrayPtr loopMsg);

    void performLoopClosure();

    bool detectLoopClosureDistance(int *latestID, int *closestID);

    bool detectLoopClosureAdditional(int *latestID, int *closestID);

    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum);

    void visualizeLoopClosure();

    void updateInitialGuess();

    void extractForLoopClosure();

    void extractNearby();

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract);

    void extractSurroundingKeyFrames();

    void downsampleCurrentScan();

    void updatePointAssociateToMap();

    void surfOptimization();

    void combineOptimizationCoeffs();

    bool LMOptimization(int iterCount);

    float F2dist(Eigen::Quaternionf &q1, Eigen::Quaternionf &q2);

    float approximateYaw(Eigen::Quaternionf &q);

    float pi = 3.14159265358;

    float wrapAngle(float angle);

    std::tuple<double, double, double> quatToRPY(const Eigen::Quaternionf& q);

    float yawDist(Eigen::Quaternionf &q1, Eigen::Quaternionf &q2);

    float quatToAngle(Eigen::Quaternionf &q);

    float angleDist(Eigen::Quaternionf &q1, Eigen::Quaternionf &q2);

    void scan2MapOptimization();

    void publishTime(float *transform, double time);

    void transformUpdate();

    float constraintTransformation(float value, float limit);

    bool saveFrame();

    void addOdomFactor();

    void addOdomAdditionalFactor(gtsam::Pose3 &poseToFirst);

    void addGPSFactor();

    void addLoopFactor();

    void saveKeyFramesAndFactor();

    void correctPoses();

    void updatePath(const PointTypePose &pose_in);

    void publishOdometry();

    void publishFrames();
};

#endif
