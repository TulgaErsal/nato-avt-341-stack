#include "avt_341/perception/slam/imu_preintegration.h"

using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)


std::shared_ptr<avt_341::node::NodeProxy> node_proxy = nullptr;

std::shared_ptr<TransformFusion> globalTF = nullptr;
void globalTF_imuOdometryHandler(avt_341::msg::OdometryPtr odomMsg) {globalTF->imuOdometryHandler(odomMsg);}
void globalTF_lidarOdometryHandler(avt_341::msg::OdometryPtr odomMsg) {globalTF->lidarOdometryHandler(odomMsg);}

std::shared_ptr<IMUPreintegration> globalImuP = nullptr;
void globalImuP_imuHandler(avt_341::msg::ImuPtr imuMsg)
{
  //node_proxy->log_info(">>> callback imu at %lf with ts %lf",node_proxy->get_now_seconds(),avt_341::node::seconds_from_header(imuMsg->header));
  globalImuP->imuHandler(imuMsg);
  //node_proxy->log_info("<<< callback imu done");
}
void globalImuP_odometryHandler(avt_341::msg::OdometryPtr odometryMsg){globalImuP->odometryHandler(odometryMsg);}

TransformFusion::TransformFusion(std::shared_ptr<avt_341::node::NodeProxy> _node_proxy) : ParamServer(_node_proxy)
    {
        _node_proxy->initialize_tf_listener();

        if (lidarFrame != baselinkFrame)
        {

            while (!lidar2Baselink_initialized)
            {
                try
                {
                  avt_341::msg::TransformStamped tfs = _node_proxy->lookup_transform(lidarFrame, baselinkFrame, avt_341::msg::Time(0));
                  lidar2Baselink = to_tf_transform(tfs);
                  //TODO invalid -> throw
                }
                catch(std::exception const & ex)
                {
                    _node_proxy->log_error("%s", ex.what());
                    _node_proxy->log_error("Cannot get initial transform from %s to %s, retrying.", lidarFrame.c_str(), baselinkFrame.c_str());
                    _node_proxy->log_error("Please make sure the TF is published before running the node.");
                    _node_proxy->log_error("Or you can change the lidarFrame and baselinkFrame in the configuration file to point to the same frame.");
                    sleep(1);
                }

            }
        }
        else
        {
            lidar2Baselink.setIdentity();
            lidar2Baselink_initialized = true;
        }

        //subLaserOdometry = nh.subscribe<avt_341::msg::Odometry>("avt_341/slam/mapping/odometry", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay()); //TODO tcpNoDelay
        //subImuOdometry = nh.subscribe<avt_341::msg::Odometry>(odomTopic + "_incremental", 2000, &TransformFusion::imuOdometryHandler, this, ros::TransportHints().tcpNoDelay()); //TODO tcpNoDelay

        //pubImuOdometry = node_proxy->create_publisher<avt_341::msg::Odometry>(odomTopic, 2000);
    }

Eigen::Affine3f TransformFusion::odom2affine(avt_341::msg::Odometry odom)
{
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        avt_341::msg_tf::Quaternion orientation = to_tf_quaterion(odom.pose.pose.orientation);
        avt_341::msg_tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

void TransformFusion::lidarOdometryHandler(avt_341::msg::OdometryPtr odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        lidarOdomAffine = odom2affine(*odomMsg);

        lidarOdomTime = avt_341::node::seconds_from_header(odomMsg->header);
        if (!imuType)
        {
            broadcastTransformation(*odomMsg);
        }
    }

void TransformFusion::broadcastTransformation(avt_341::msg::Odometry odomMsg)
    {
        if (lidar2Baselink_initialized == false)
        {
            return;
        }
        node_proxy->initialize_tf_listener();

        avt_341::msg_tf::Transform tCur = pose_to_transform(odomMsg.pose.pose);
        if (lidarFrame != baselinkFrame)
            tCur = tCur * lidar2Baselink;
        avt_341::msg::Time tstmp = odomMsg.header.stamp;
        avt_341::msg::TransformStamped odom_2_baselink = from_tf_transform(tCur, tstmp, odometryFrame, baselinkFrame);
        node_proxy->broadcast_tf(odom_2_baselink);

    }

void TransformFusion::imuOdometryHandler(avt_341::msg::OdometryPtr odomMsg)
    {
        // static tf
        //static tf::TransformBroadcaster tfMap2Odom;
        //static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        //tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, odomMsg->header.stamp, mapFrame, odometryFrame));
        static avt_341::msg_tf::Transform map_to_odom(
            avt_341::msg_tf::Quaternion(0, 0, 0, 1),
            avt_341::msg_tf::Vector3(0, 0, 0)
            );

        avt_341::msg::Time ts = odomMsg->header.stamp;
        avt_341::msg::TransformStamped map_2_odom = from_tf_transform(map_to_odom, ts, mapFrame, odometryFrame);
        node_proxy->broadcast_tf(map_2_odom);

        std::lock_guard<std::mutex> lock(mtx);

        imuOdomQueue.push_back(*odomMsg);

        // get latest odometry (at current IMU stamp)
        if (lidarOdomTime == -1)
            return;
        while (!imuOdomQueue.empty())
        {
            if (avt_341::node::seconds_from_header(imuOdomQueue.front().header) <= lidarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }
        Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
        Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());
        Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
        Eigen::Affine3f imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);

        // publish latest odometry
        avt_341::msg::Odometry laserOdometry = imuOdomQueue.back();
        laserOdometry.pose.pose.position.x = x;
        laserOdometry.pose.pose.position.y = y;
        laserOdometry.pose.pose.position.z = z;
        //laserOdometry.pose.pose.orientation = avt_341::msg_tf::Quaternion(roll, pitch, yaw); //TODO check RPY vs YPR
        avt_341::msg_tf::Quaternion qtmp;
        qtmp.setRPY(roll,pitch,yaw);
        laserOdometry.pose.pose.orientation = from_tf_quaternion(qtmp);
        pubImuOdometry->publish(laserOdometry);

        // publish tf
        // if (lidar2Baselink_initialized)
        // {
        //     static tf::TransformBroadcaster tfOdom2BaseLink;
        //     tf::Transform tCur;
        //     tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
        //     if (lidarFrame != baselinkFrame)
        //         tCur = tCur * lidar2Baselink;
        //     tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame);
        //     tfOdom2BaseLink.sendTransform(odom_2_baselink);
        // }
        if (imuType)
        {
            broadcastTransformation(laserOdometry);
        }

        // publish IMU path
        static avt_341::msg::Path imuPath;
        static double last_path_time = -1;
        double imuTime = avt_341::node::seconds_from_header(imuOdomQueue.back().header);
        if (imuTime - last_path_time > 0.1)
        {
            last_path_time = imuTime;
            avt_341::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = odometryFrame;
            pose_stamped.pose = laserOdometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            while (!imuPath.poses.empty() && avt_341::node::seconds_from_header(imuPath.poses.front().header) < lidarOdomTime - 1.0)
                imuPath.poses.erase(imuPath.poses.begin());
            //if (pubImuPath.getNumSubscribers() != 0) #TODO
            if (true)
            {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = odometryFrame;
                pubImuPath->publish(imuPath);
            }
        }
    }


IMUPreintegration::IMUPreintegration(std::shared_ptr<avt_341::node::NodeProxy> _node_proxy) : ParamServer(_node_proxy)
    {
        //subImu = nh.subscribe<avt_341::msg::Imu>(imuTopic, 2000, &IMUPreintegration::imuHandler, this, ros::TransportHints().tcpNoDelay()); //TODO tcpNoDelay()
        //subOdometry = nh.subscribe<avt_341::msg::Odometry>("avt_341/slam/mapping/odometry_incremental", 5, &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay()); //TODO tcpNoDelay()

        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuGyrNoise, 2);     // gyro white noise in continuous
        p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2);          // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());
        ; // assume zero initial bias

        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);                                                               // m/s
        priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);                                                             // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());   // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());                 // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization
    }

void IMUPreintegration::resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

void IMUPreintegration::resetParams()
    {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

void IMUPreintegration::odometryHandler(avt_341::msg::OdometryPtr odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        double currentCorrectionTime = ROS_TIME(odomMsg);

        // make sure we have imu data to integrate
        if (imuQueOpt.empty())
            return;

        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));

        // 0. initialize system
        if (systemInitialized == false)
        {
            if (degenerate)
            {
                node_proxy->log_error("Can't Initialize IMU with a degenerate odometry!");
                return;
            }
            resetOptimization();

            // pop old IMU message
            while (!imuQueOpt.empty())
            {
                if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
                {
                    lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }
            // initial pose
            prevPose_ = lidarPose.compose(lidar2Imu);
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);
            // initial velocity
            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);
            // initial bias
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

            key = 1;
            systemInitialized = true;
            return;
        }

        // reset graph for speed
        if ((key == maxIMUgraphLen))
        {
            // node_proxy->log_info("RESETTING IMU GRAPH");
            // get updated noise before reset
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key - 1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key - 1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key - 1)) / 5);
            // reset graph
            resetOptimization();
            // add pose
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            // add velocity
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
            // add bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            key = 1;
        }

        // 1. integrate imu data and optimize
        while (!imuQueOpt.empty())
        {
            // pop and integrate imu data that is between two optimizations
            avt_341::msg::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            if (imuTime < currentCorrectionTime - delta_t)
            {
                double dt = (lastImuT_opt < 0) ? (1.0 / imuRate) : (imuTime - lastImuT_opt);
                if (dt <= 0)
                {
                    node_proxy->log_warning("dt <= 0 in imu integration");
                    // lastImuT_opt = imuTime;
                    imuQueOpt.pop_front();
                    continue;
                }
                imuIntegratorOpt_->integrateMeasurement(
                    gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                    gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z), dt);

                lastImuT_opt = imuTime;
                imuQueOpt.pop_front();
            }
            else
                break;
        }
        // add imu factor to graph
        const gtsam::PreintegratedImuMeasurements &preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        // add imu bias between factor
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                                                                            gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
        // add pose factor
        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);
        // insert predicted values
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);
        // optimize
        optimizer.update(graphFactors, graphValues);
        optimizer.update();
        graphFactors.resize(0);
        graphValues.clear();
        // Overwrite the beginning of the preintegration for the next step.
        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_ = result.at<gtsam::Pose3>(X(key));
        prevVel_ = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
        // Reset the optimization preintegration object.
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        // check optimization
        if (failureDetection(prevVel_, prevBias_))
        {
            resetParams();
            return;
        }

        // 2. after optiization, re-propagate imu odometry preintegration
        prePrevStateOdom = prevStateOdom;
        prevStateOdom = prevState_;
        if (!imuType)
        {
            prevStateOdom = gtsam::NavState(prevState_.pose().rotation(), gtsam::Point3(p_x, p_y, p_z), prevState_.velocity());
        }

        prevBiasOdom = prevBias_;
        // first pop imu message older than current correction data
        double lastImuQT = -1;
        while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }
        // repropogate
        if (!imuQueImu.empty())
        {
            // reset bias use the newly optimized bias
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                avt_341::msg::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / imuRate) : (imuTime - lastImuQT);
                if (dt <= 0)
                {
                    node_proxy->log_warning("dt <= 0 in imu integration");
                    lastImuQT = imuTime;
                    continue;
                }

                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

        ++key;
        doneFirstOpt = true;
    }

bool IMUPreintegration::failureDetection(const gtsam::Vector3 &velCur, const gtsam::imuBias::ConstantBias &biasCur)
    {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30)
        {
            node_proxy->log_warning("Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            node_proxy->log_warning("Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

void IMUPreintegration::imuHandler(avt_341::msg::ImuPtr imu_raw)
    {
        std::lock_guard<std::mutex> lock(mtx);

        avt_341::msg::Imu thisImu = imuConverter(*imu_raw);

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        if (doneFirstOpt == false)
            return;

        double imuTime = ROS_TIME(&thisImu);
        double dt = (lastImuT_imu < 0) ? (1.0 / imuRate) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;

        if (dt <= 0)
        {
            node_proxy->log_warning("dt <= 0 in imu handler");
            return;
        }

        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                                gtsam::Vector3(thisImu.angular_velocity.x, thisImu.angular_velocity.y, thisImu.angular_velocity.z), dt);

        // predict odometry
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        // publish odometry
        avt_341::msg::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = odometryFrame;
        odometry.child_frame_id = "odom_imu";

        // transform imu pose to ldiar
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        if (imuType)
        {
            odometry.pose.pose.position.x = lidarPose.translation().x();
            odometry.pose.pose.position.y = lidarPose.translation().y();
            odometry.pose.pose.position.z = lidarPose.translation().z();
        }
        else
        {
            auto previous_translation = prevStateOdom.pose().translation() - prePrevStateOdom.pose().translation();
            // odometry.pose.pose.position.x = prevStateOdom.pose().translation().x();
            // odometry.pose.pose.position.y = prevStateOdom.pose().translation().y();
            // odometry.pose.pose.position.z = prevStateOdom.pose().translation().z();
            odometry.pose.pose.position.x = previous_translation.x() + prevStateOdom.pose().translation().x();
            odometry.pose.pose.position.y = previous_translation.y() + prevStateOdom.pose().translation().y();
            odometry.pose.pose.position.z = previous_translation.z() + prevStateOdom.pose().translation().z();
        }

        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();

        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        pubImuOdometry->publish(odometry);
        //node_proxy->log_info("pub odom");
    }

int main(int argc, char **argv)
{
    node_proxy = avt_341::node::init_node(argc, argv, "imu_preintegration");
 
    globalTF = std::make_shared<TransformFusion>(node_proxy);
    globalTF->pubImuOdometry = node_proxy->create_publisher<avt_341::msg::Odometry>(globalTF->odomTopic + "_incremental", 2000);
    globalTF->pubImuPath = node_proxy->create_publisher<avt_341::msg::Path>("avt_341/slam/imu/path", 1);
    auto globalTF_subLaserOdometry = node_proxy->create_subscription<avt_341::msg::Odometry>("avt_341/slam/mapping/odometry", 5, globalTF_lidarOdometryHandler);
    auto globalTF_subImuOdometry = node_proxy->create_subscription<avt_341::msg::Odometry>(globalTF->odomTopic + "_incremental", 2000, globalTF_imuOdometryHandler);
   
    globalImuP = std::make_shared<IMUPreintegration>(node_proxy);
    globalImuP->pubImuOdometry = node_proxy->create_publisher<avt_341::msg::Odometry>(globalImuP->odomTopic, 2000);
    auto globalImuP_subImu = node_proxy->create_subscription<avt_341::msg::Imu>(globalImuP->imuTopic, 2000, globalImuP_imuHandler);
    auto globalImuP_subOdometry = node_proxy->create_subscription<avt_341::msg::Odometry>("avt_341/slam/mapping/odometry_incremental", 5, globalImuP_odometryHandler);

    node_proxy->log_info("\033[1;32m----> IMU Preintegration Started.\033[0m");

    //ros::MultiThreadedSpinner spinner(4); TODO
    //spinner.spin();
    node_proxy->spin();


    return 0;
}
