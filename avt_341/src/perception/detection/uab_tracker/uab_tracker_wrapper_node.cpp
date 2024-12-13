#include <vector>
#include <array>
#include <math.h>

#include "avt_341/perception/detection/uab_tracker/lib_tracker_wrapper.h"
#include "mclcppclass.h"
#include "mclmcrrt.h"

#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include <tf2/LinearMath/Quaternion.h>


avt_341::msg::Odometry odom;

bool odomReceived = false;

avt_341::msg::PointCloud2 pc;

bool pcReceived = false;

//autr4k_msgs::Detection detection;
avt_341::msg::Detection2d  detection;

bool detectionReceived = false;

void OdometryCallback(avt_341::msg::OdometryPtr rcvOdom) {
  odom = *rcvOdom;
  odomReceived = true;
}

void PointCloudCallback(avt_341::msg::PointCloud2Ptr rcvPc) {
  ROS_INFO_THROTTLE(1, "pc received");
  pc = *rcvPc;
  pcReceived = true;
}

void DetectionCallback(const avt_341::msg::Detection2d::ConstPtr& rcvDetection) {
  detection = *rcvDetection;
  detectionReceived = true;
}

bool allMsgsReceived() {
  return odomReceived && pcReceived && detectionReceived;
}

void GetVehiclePosition(bool& leaderFound,
                        std::vector<double>& leaderDetectedPosition,
                        std::vector<double>& leaderFilteredState) {
  //
  //matlab only accepts primitives, so we have to deconstruct ROS messages
  //
  //raw pointcloud
  std::vector <uint8_t> rawPcData(std::begin(pc.data), std::end(pc.data));
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
  mwArray detectionScore(detection.hypothesis.score);

  //camera bounding box is in full image resolution frame
  std::vector <int64_t> cameraBoundingBox
    {detection.bounding_box.x_min, detection.bounding_box.y_min,
     detection.bounding_box.x_max, detection.bounding_box.y_max};
  mwArray mwCameraBoundingBox(1, cameraBoundingBox.size(), mxINT64_CLASS);
  mwCameraBoundingBox.SetData(&cameraBoundingBox.front(), cameraBoundingBox.size());

  try {
    //output variables
    mwArray mwLeaderDetected;
    mwArray mwLeaderDetectedPosition;
    mwArray mwLeaderFilteredState;
    trackerWrapper(3,                                                  //number of output arguments
                   mwLeaderDetected,                                  //output variable
                   mwLeaderDetectedPosition,                          //output variable
                   mwLeaderFilteredState,                             //output variable
                   pcWidth, pcHeight, pcPointStep, pcRowStep, pcData, //pointcloud
                   detectionScore, mwCameraBoundingBox,                //detection
                   x, y, z, qw, qx, qy, qz);                          //odometry
    std::cout << "trackerwrapper done" << std::endl;
    //parse output
    leaderFound = mwLeaderDetected; //vehicle detected?
    std::cout << "Leader? " << leaderFound << std::endl;

    if (leaderFound) {
      //detected leader position
      uint8_t size = 2; //x, y
      std::vector<double> leaderPos(size);
      mwLeaderDetectedPosition.GetData(leaderPos.data(), leaderPos.size());
      leaderDetectedPosition = leaderPos;
      std::cout << "detectedPos: " << leaderDetectedPosition[0] << std::endl;

      //filtered leader position
      uint8_t filteredSize = 4; //x, y, theta, speed
      std::vector<double> filteredState(filteredSize);
      mwLeaderFilteredState.GetData(filteredState.data(), filteredState.size());
      leaderFilteredState = filteredState;
      std::cout << "filteredPos: " << filteredState[0] << std::endl;
    }

  } catch (const mwException& e) {
    std::cerr << "Error" << std::endl;
    std::cerr << "dscore: " << detectionScore << ", boundingbox: " << mwCameraBoundingBox
              << "\npos: " << x << ", " << y << ", " << z
              << "\nrot: " << qw << ", " << qx << ", " << qy << ", " << qz << std::endl;
    std::cerr << "Failed to execute Matlab tracker_wrapper. " << e.what() << std::endl;
  }
}

int main(int argc, char* argv[]) {
  //initialize matlab runtime
  if (!mclInitializeApplication(NULL, 0)) {
    std::cerr << "Failed to initialize Matlab Runtime" << std::endl;
    return -1;
  }

  //initialize DLL
  if (!lib_tracker_wrapperInitialize()) {
    std::cerr << "Failed to initialize tracker_wrapper package" << std::endl;
    return -1;
  }

  auto node = avt_341::node::init_node(argc, argv, "uab_tracker_wrapper_node");
  auto sub_odom =
    node->create_subscription<avt_341::msg::Odometry>("/warpath/navigation/odometry_local_center_enu",
                                                      1,
                                                      OdometryCallback);
  auto sub_pointcloud =
    node->create_subscription<avt_341::msg::PointCloud2>("/ugv_sensors/lidar/cloud/points", 1, PointCloudCallback);
  auto sub_detection =
    node->create_subscription<avt_341::msg::Detection2d>("/autr4k/detection/detection", 10, DetectionCallback);

  auto pub_leader_detection = node->create_publisher<avt_341::msg::Odometry>("avt_341/leader_detection", 10);
  auto pub_leader_filtered = node->create_publisher<avt_341::msg::Odometry>("avt_341/leader_filtered", 10);

  avt_341::node::Rate rate(100.0);
  while (avt_341::node::ok()) {
    if (!allMsgsReceived()) {
      std::string waitingOn = "waiting on ";
      if (!odomReceived) waitingOn += "odom ";
      if (!pcReceived) waitingOn += "pointcloud ";
      if (!detectionReceived) waitingOn += "detection";
      std::cout << waitingOn << std::endl;

      avt_341::node::Rate wait(1.0);
      wait.sleep();
    } else {
      bool leaderDetected;
      std::vector<double> leaderDetectedState;
      std::vector<double> leaderFilteredState;
      GetVehiclePosition(leaderDetected, leaderDetectedState, leaderFilteredState);
      if (leaderDetected) {
        std::cout << leaderDetectedState[0] << "," << leaderDetectedState[1] << std::endl;
        std::cout << leaderFilteredState[0] << "," << leaderFilteredState[1] << ", heading: " << leaderFilteredState[2]
                  << ", v:" << leaderFilteredState[3] << std::endl;

        avt_341::msg::Odometry leader_detection;
        leader_detection.header.frame_id = "map";
        leader_detection.child_frame_id = "test_detection";
        leader_detection.pose.pose.position.x = leaderDetectedState[0];
        leader_detection.pose.pose.position.y = leaderDetectedState[1];
        leader_detection.pose.pose.position.z = odom.pose.pose.position.z;

        avt_341::msg::Odometry leader_filtered;
        leader_filtered.header.frame_id = "map";
        leader_filtered.child_frame_id = "test_filtered";
        leader_filtered.pose.pose.position.x = leaderFilteredState[0];
        leader_filtered.pose.pose.position.y = leaderFilteredState[1];
        leader_filtered.pose.pose.position.z = odom.pose.pose.position.z;
        leader_filtered.twist.twist.linear.x = leaderFilteredState[3];

        double heading = leaderFilteredState[2];  // in radians
        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, heading);  // Roll, Pitch, Yaw
        leader_filtered.pose.pose.orientation.x = quaternion.x();
        leader_filtered.pose.pose.orientation.y = quaternion.y();
        leader_filtered.pose.pose.orientation.z = quaternion.z();
        leader_filtered.pose.pose.orientation.w = quaternion.w();


        pub_leader_detection->publish(leader_detection);
        pub_leader_filtered->publish(leader_filtered);
      } else {
        std::cout << "Leader not detected" << std::endl;
      }

    }
    node->spin_some();
    rate.sleep();
  }

  mclTerminateApplication();
  return 0;
}
