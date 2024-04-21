/* avt_341_lidar_occlusion_detector_node.cpp

 * ROS Node for detecting occlusions of lidar sensor data
 * Subscribers: points (PointCloud2)
 * Publishers:  gt_occ_detected (int32)

**/

// c++ includes
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
// ros includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"

