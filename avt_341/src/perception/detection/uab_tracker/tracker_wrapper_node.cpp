#include <vector>
#include <array>
#include <math.h>

#include "mclcppclass.h"
#include "mclmcrrt.h"

#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "autr4k/Detection/Detector.hpp"
#include "lib_tracker_wrapper.h"

avt_341::msg::Odometry odom;
bool odomReceived = false;

avt_341::msg::PointCloud2 pc;
bool pcReceived = false;

autr4k_msgs::msg::Detection detection;
bool detectionReceived = false;

void OdometryCallback(avt_341::msg::OdometryPtr rcvOdom)
{
    odom = *rcvOdom;
    odomReceived = true;
}

void PointCloudCallback(avt_341::msg::PointCloud2Ptr rcvPc)
{
    pc = *rcvPc;
    pcReceived = true;
}

void DetectionCallback(autr4k_msgs::msg::Detection rcvDetection)
{
    detection = *rcvDetection;
    detectionReceived = true;
}

bool allMsgsReceived()
{
    return odomReceived && pcReceived && detectionReceived;
}

std::vector<double> GetVehiclePosition(bool& mrzrFound)
{
    //
    //matlab only accepts primitives, so we have to deconstruct ROS messages
    //
    //raw pointcloud
    std::vector<uint8_t> rawPcData(std::begin(pc.data), std::end(pc.data));
    mwArray pcData(1, rawPcData.size(), mxUINT8_CLASS);
    pcData.SetData(&rawPcData.front(), rawPcData.size());

    //pointcloud properties
    mwArray pcWidth(pc.width);
    mwArray pcHeight(pc.height);
    mwArray pcPointStep(pc.point_step);
    mwArray pcRowStep(pc.row_step);

    //odometry
    mwArray x(odom.pose.pose.position.x);
    mwArray y(odom.pose.pose.position.y);
    mwArray z(odom.pose.pose.position.z);
    mwArray qw(odom.pose.pose.orientation.w);
    mwArray qx(odom.pose.pose.orientation.x);
    mwArray qy(odom.pose.pose.orientation.y);
    mwArray qz(odom.pose.pose.orientation.z);

    //detection
    mwArray xMinBB(detection.hypotheses[0].bounding_box.x_min);
    mwArray xMaxBB(detection.hypotheses[0].bounding_box.x_max);
    mwArray yMinBB(detection.hypotheses[0].bounding_box.y_max);
    mwArray yMaxBB(detection.hypotheses[0].bounding_box.y_min);
    mwArray detectionScore(detection.hypotheses[0].score);

    try
    {
        //output variables
        mwArray mwVehiclePosition;
        mwArray mwVehicleFound;
        trackerWrapper(2,                                                  //number of output arguments
                        mwVehicleFound,                                    //output variable
                        mwVehiclePosition,                                 //output variable
                        pcData, pcWidth, pcHeight, pcPointStep, pcRowStep, //pointcloud
                        xMinBB, yMinBB, xMaxBB, yMaxBB, detectionScore,    //detection
                        x, y, z, qw, qx, qy, qz);                          //odometry

        //parse output
        bool vehicleFound = mwVehicleFound; //vehicle detected?

        uint8_t size = 2; //x, y
        std::vector<double> vehiclePos(size);
        mwVehiclePosition.GetData(vehiclePos.data(), vehiclePos.size());
        return vehiclePos;
    }
    catch(const mwException& e)
    {
        std::cerr << "Failed to execute Matlab tracker_wrapper. " << e.what() << std::endl;
    }
}

int main(int argc, char *argv[])
{
    //initialize matlab runtime
    if (!mclInitializeApplication(NULL, 0))
    {
        std::cerr << "Failed to initialize Matlab Runtime" << std::endl;
        return -1;
    }

    //initialize DLL
    if(!lib_tracker_wrapperInitialize())
    {
        std::cerr << "Failed to initialize tracker_wrapper package" << std::endl;
        return -1;
    }
    
    avt_341::node::Rate rate(100.0);
    while (avt_341::node::ok())
    {
        if (!allMsgsReceived())
        {
            std::string waitingOn = "waiting on ";
            if (!odomReceived) waitingOn += "odom ";
            if (!pcReceived) waitingOn += "pointcloud ";
            if (!detectionReceived) waitingOn += "detection";
            std::cout << waitingOn << std::endl;
            
            avt_341::node::Rate wait(1.0);
            wait.sleep();
        }
        else
        {
            std::vector<double> vehiclePos = GetVehiclePosition();
            std::cout << vehiclePos[0] << "," << vehiclePos[1] << std::endl;
        }
        node->spin_some();
        rate.sleep();
    }

    mclTerminateApplication();
    return 0;
}
