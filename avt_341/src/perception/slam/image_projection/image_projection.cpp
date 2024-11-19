#include "avt_341/perception/slam/image_projection.h"
// <!-- liorf_localization_yjz_lucky_boy -->

std::shared_ptr<avt_341::node::NodeProxy> node_proxy = nullptr;

std::shared_ptr<ImageProjection> globalIP = nullptr;
void globalIP_imuHandler(avt_341::msg::ImuPtr imuMsg) {
  //node_proxy->log_info(">>> callback imu at %lf with ts %lf",node_proxy->get_now_seconds(),avt_341::node::seconds_from_header(imuMsg->header));
  globalIP->imuHandler(imuMsg);
  //node_proxy->log_info("<<< callback imu done");
}
void globalIP_odometryHandler(avt_341::msg::OdometryPtr odomMsg) {
  //node_proxy->log_info(">>> callback odom at %lf with ts %lf",node_proxy->get_now_seconds(),avt_341::node::seconds_from_header(odomMsg->header));
  globalIP->odometryHandler(odomMsg);
  //node_proxy->log_info("<<< callback odom done");
}
void globalIP_cloudHandler(avt_341::msg::PointCloud2Ptr pc2Msg) {
  //node_proxy->log_info(">>> callback pc2 at %lf with ts %lf",node_proxy->get_now_seconds(),avt_341::node::seconds_from_header(pc2Msg->header));
  globalIP->cloudHandler(pc2Msg);
  //node_proxy->log_info("<<< callback pc2 done");
}

ImageProjection::ImageProjection(std::shared_ptr<avt_341::node::NodeProxy> node_proxy_) : deskewFlag(0), ParamServer(node_proxy_)
    {
        if (imuType || (!calculateInitRollPitch)){
            imuReceived = true;
        } else {
            imuReceived = false;
        }


        node_proxy->log_info("useIMU: %d", useIMU);

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

void ImageProjection::allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        tmpMulranCloudIn.reset(new pcl::PointCloud<MulranPointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());

        resetParameters();
    }

void ImageProjection::resetParameters()
    {
        laserCloudIn->clear();
        fullCloud->clear();

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }

    }

void ImageProjection::imuHandler(avt_341::msg::ImuPtr imuMsg)
    {
        avt_341::msg::Imu thisImu = imuConverter(*imuMsg);

        // calculate initial roll and pitch angle using the first few IMU messages
        
        static int measurements_received = 0;
        static Eigen::Vector3d accelerationVector;
        static double roll = 0, pitch = 0;
        if ((calculateInitRollPitch) && (!imuType))
        {
            Eigen::Vector3d acc;
            acc << thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z;
            acc.normalize();
            accelerationVector = (accelerationVector * measurements_received + acc) / (measurements_received + 1);
            measurements_received++;
            if (measurements_received > initRollPitchWindow)
            {
                accelerationVector = accelerationVector;
                std::cout << "acceleration vector: " << accelerationVector.transpose() << std::endl;
                calculateInitRollPitch = false;
                roll = atan2(accelerationVector(1), accelerationVector(2));
                pitch = asin(-accelerationVector(0));
                initRoll = roll;
                initPitch = pitch;
                node_proxy->log_info("initial roll: %f, pitch: %f", roll, pitch);
                imuReceived = true;
            }
        } else {
            std::lock_guard<std::mutex> lock1(imuLock);
            imuQueue.push_back(thisImu);
        }        
    }

void ImageProjection::odometryHandler(avt_341::msg::OdometryPtr odometryMsg)
    {
        if (!imuReceived)
        {
            //ROS_INFO_THROTTLE(1000, "IMU not received, aborting scan");
            node_proxy->log_info("IMU not received, aborting scan");
            return;
        }

        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

void ImageProjection::cloudHandler(avt_341::msg::PointCloud2Ptr laserCloudMsg)
    {
        if (!imuReceived)
        {
            //ROS_INFO_THROTTLE(1000, "IMU not received, aborting scan");
            node_proxy->log_info("IMU not received, aborting scan");
            return;
        }
        if (!cachePointCloud(laserCloudMsg)){
            node_proxy->log_info("Cloud not cached, aborting scan");
            return;
        }
        static int cloudCounter = 0;
        static int deskewCounter = 0;
        cloudCounter++;
        if (!deskewInfo()){
            deskewCounter++;
            //ROS_INFO_THROTTLE(10, "Deskew info not available, aborting scan (%d/%d)", deskewCounter, cloudCounter);
            node_proxy->log_info("Deskew info not available, aborting scan (%d/%d)", deskewCounter, cloudCounter);
            return;
        }

        projectPointCloud();

        publishClouds();

        resetParameters();
    }

bool ImageProjection::cachePointCloud(avt_341::msg::PointCloud2Ptr laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 0)
            return false;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();
        if (sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX)
        {
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
        }
        else if (sensor == SensorType::OUSTER)
        {
            // Convert to Velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            laserCloudIn->width = tmpOusterCloudIn->width;
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.t * 1e-9f;
            }
        } // <!-- liorf_yjz_lucky_boy -->
        else if (sensor == SensorType::MULRAN)
        {
            // Convert to Velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpMulranCloudIn);
            laserCloudIn->points.resize(tmpMulranCloudIn->size());
            laserCloudIn->is_dense = tmpMulranCloudIn->is_dense;
            for (size_t i = 0; i < tmpMulranCloudIn->size(); i++)
            {
                auto &src = tmpMulranCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = static_cast<float>(src.t);
            }
        } // <!-- liorf_yjz_lucky_boy -->
        else if (sensor == SensorType::ROBOSENSE) {
            pcl::PointCloud<RobosensePointXYZIRT>::Ptr tmpRobosenseCloudIn(new pcl::PointCloud<RobosensePointXYZIRT>());
            // Convert to robosense format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpRobosenseCloudIn);
            laserCloudIn->points.resize(tmpRobosenseCloudIn->size());
            laserCloudIn->is_dense = tmpRobosenseCloudIn->is_dense;

            double start_stamptime = tmpRobosenseCloudIn->points[0].timestamp;
            for (size_t i = 0; i < tmpRobosenseCloudIn->size(); i++) {
                auto &src = tmpRobosenseCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.timestamp - start_stamptime;
            }
        } 
        else {
            //ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            node_proxy->log_error("Unknown sensor type: %d", int(sensor));
            node_proxy->shutdown();
        }

        // get timestamp
        cloudHeader = currentCloudMsg.header;
        timeScanCur = avt_341::node::seconds_from_header(cloudHeader);
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        // check dense flag
        if (laserCloudIn->is_dense == false)
        {
            node_proxy->log_error("Point cloud is not in dense format, please remove NaN points first!");
            node_proxy->shutdown();
        }

        // check ring channel
        static int ringFlag = 0;
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1)
            {
                // ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                // ros::shutdown();
            }
        }

        // check point time
        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (auto &field : currentCloudMsg.fields)
            {
                if (field.name == "time" || field.name == "t")
                {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1)
                node_proxy->log_warning("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        return true;
    }

bool ImageProjection::deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan
        if ( useIMU && (imuQueue.empty() || avt_341::node::seconds_from_header(imuQueue.front().header) > timeScanCur || avt_341::node::seconds_from_header(imuQueue.back().header) < timeScanEnd) )
        {
            //ROS_INFO_THROTTLE(10, "Waiting for IMU data ...");
            node_proxy->log_info("Waiting for IMU data ...");
            return false;
        }

        imuDeskewInfo();

        odomDeskewInfo();

        return true;
    }

void ImageProjection::imuDeskewInfo()
    {
        cloudInfo.imu_available = false;

        while (!imuQueue.empty())
        {
            if (avt_341::node::seconds_from_header(imuQueue.front().header) < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            avt_341::msg::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = avt_341::node::seconds_from_header(thisImuMsg.header);

            if (imuType) {
                // get roll, pitch, and yaw estimation for this scan
                if (currentImuTime <= timeScanCur){
                    imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imu_roll_init, &cloudInfo.imu_pitch_init, &cloudInfo.imu_yaw_init);
                }
            } else {
                 cloudInfo.imu_roll_init = initRoll;
                 cloudInfo.imu_pitch_init = initPitch;
            }

            if (currentImuTime > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imu_available = true;
    }

void ImageProjection::odomDeskewInfo()
    {
        cloudInfo.odom_available = false;
        static float sync_diff_time = (imuRate >= 300) ? 0.01 : 0.20;
        while (!odomQueue.empty())
        {
            if (avt_341::node::seconds_from_header(odomQueue.front().header) < timeScanCur - sync_diff_time)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty()){
            //ROS_WARN_THROTTLE(10, "no odom");
            node_proxy->log_warning("no odom");
            return;
        } 

        if (avt_341::node::seconds_from_header(odomQueue.front().header) > timeScanCur){
            //ROS_WARN_THROTTLE(10, "odom start time is ahead of scan time");
            node_proxy->log_warning("odom start time is ahead of scan time");
            return;
        }

        // get start odometry at the beinning of the scan
        avt_341::msg::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        avt_341::msg_tf::Quaternion orientation = to_tf_quaterion(startOdomMsg.pose.pose.orientation);

        double roll, pitch, yaw;
        avt_341::msg_tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        cloudInfo.initial_guess_x = startOdomMsg.pose.pose.position.x;
        cloudInfo.initial_guess_y = startOdomMsg.pose.pose.position.y;
        cloudInfo.initial_guess_z = startOdomMsg.pose.pose.position.z;
        cloudInfo.initial_guess_roll  = roll;
        cloudInfo.initial_guess_pitch = pitch;
        cloudInfo.initial_guess_yaw   = yaw;

        cloudInfo.odom_available = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (avt_341::node::seconds_from_header(odomQueue.back().header) < timeScanEnd){
            //ROS_WARN_THROTTLE(10, "odom end time is behind of scan time");
            node_proxy->log_warning("odom end time is behind of scan time");
            return;
        }

        avt_341::msg::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanEnd)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        orientation = to_tf_quaterion(endOdomMsg.pose.pose.orientation);
        avt_341::msg_tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

void ImageProjection::findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

void ImageProjection::findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanEnd - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

    PointType ImageProjection::deskewPoint(PointType *point, double relTime)
    {
        if (deskewFlag == -1 || cloudInfo.imu_available == false)
            return *point;

        double pointTime = timeScanCur + relTime;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

void ImageProjection::projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();
        int width = laserCloudIn->width;
        // range image projection
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = common_lib_->pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            int rowIdn = i/width;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            if (i % point_filter_num != 0)
                continue;

            // check that not nans
            if (std::isnan(thisPoint.x) || std::isnan(thisPoint.y) || std::isnan(thisPoint.z))
                continue;
            // check that not inf
            if (!std::isfinite(thisPoint.x) || !std::isfinite(thisPoint.y) || !std::isfinite(thisPoint.z))
                continue;

            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            fullCloud->push_back(thisPoint);
        }
    }
    
void ImageProjection::publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed  = publishCloud(pubExtractedCloud, fullCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo->publish(cloudInfo);
    }

int main(int argc, char** argv)
{
    node_proxy = avt_341::node::init_node(argc, argv, "liorf");

    common_lib_ = std::make_shared<CommonLib::common_lib>("mapping");

    globalIP = std::make_shared<ImageProjection>(node_proxy);
    globalIP->pubExtractedCloud = node_proxy->create_publisher<avt_341::msg::PointCloud2> ("avt_341/slam/deskew/cloud_deskewed", 1);
    globalIP->pubLaserCloudInfo = node_proxy->create_publisher<avt_341::msg::LiorfCloudInfo> ("avt_341/slam/deskew/cloud_info", 1);
    //TODO transport hind tcpnodelay
    auto globalIP_subImu = node_proxy->create_subscription<avt_341::msg::Imu>(globalIP->imuTopic, 2000, globalIP_imuHandler);
    auto globalIP_subOdom = node_proxy->create_subscription<avt_341::msg::Odometry>(globalIP->odomTopic+"_incremental", 2000, globalIP_odometryHandler);
    auto globalIP_subLaserCloud = node_proxy->create_subscription<avt_341::msg::PointCloud2>(globalIP->pointCloudTopic, 5, globalIP_cloudHandler);

    node_proxy->log_info("\033[1;32m----> Image Projection Started.\033[0m");

    //ros::MultiThreadedSpinner spinner(3); TODO
    //spinner.spin();
    node_proxy->spin();
    
    return 0;
}
