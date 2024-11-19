#ifndef _IMAGE_PROJECTION_LIDAR_ODOMETRY_H_
#define _IMAGE_PROJECTION_LIDAR_ODOMETRY_H_

#include "avt_341/perception/slam/utility_node_proxy.h"

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

struct RobosensePointXYZIRT
{
    PCL_ADD_POINT4D
    float intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RobosensePointXYZIRT, 
      (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
      (uint16_t, ring, ring)(double, timestamp, timestamp)
)

// mulran datasets
struct MulranPointXYZIRT {
    PCL_ADD_POINT4D
    float intensity;
    uint32_t t;
    int ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 }EIGEN_ALIGN16;
 POINT_CLOUD_REGISTER_POINT_STRUCT (MulranPointXYZIRT,
     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
     (uint32_t, t, t) (int, ring, ring)
 )

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;

    std::deque<avt_341::msg::Imu> imuQueue;
    std::deque<avt_341::msg::Odometry> odomQueue;
    std::deque<avt_341::msg::PointCloud2> cloudQueue;
    avt_341::msg::PointCloud2 currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<MulranPointXYZIRT>::Ptr tmpMulranCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;

    int deskewFlag;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    avt_341::msg::LiorfCloudInfo cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    avt_341::msg::Header cloudHeader;

    bool imuReceived{false};
    float initRoll{0}, initPitch{0};

public:

    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::PointCloud2>> pubExtractedCloud = nullptr;
    std::shared_ptr<avt_341::node::Publisher<avt_341::msg::LiorfCloudInfo>> pubLaserCloudInfo = nullptr;

    ImageProjection(std::shared_ptr<avt_341::node::NodeProxy>);
    void allocateMemory();
    void resetParameters();
    ~ImageProjection(){}

    void imuHandler(avt_341::msg::ImuPtr imuMsg);
    void odometryHandler(avt_341::msg::OdometryPtr odometryMsg);
    void cloudHandler(avt_341::msg::PointCloud2Ptr laserCloudMsg);
    bool cachePointCloud(avt_341::msg::PointCloud2Ptr laserCloudMsg);

    bool deskewInfo();
    void imuDeskewInfo();
    void odomDeskewInfo();

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur);
    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur);

    PointType deskewPoint(PointType *point, double relTime);

    void projectPointCloud();
    void publishClouds();
};


#endif
