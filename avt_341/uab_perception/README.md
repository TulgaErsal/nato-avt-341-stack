# UAB Perception ROS2 Node

## The DLL is available [here](https://drive.google.com/file/d/1j6TEM9lfAfgaeCVbMfLkTCo0M9c7FW8X/view?usp=sharing). It has been moved because of the GitHub bandwidth and storage limits on private repositories.

## Performs semantic segmentation on camera images using the DeepLab V3+ architecture and fuses with pointcloud data from a Velodyne VLP-16 LiDAR sensor to create a costmap based on vehicle traversability. Requires MATLAB 2023a Runtime.

The `uab_perception_node` is a C++ wrapper for the machine learning model written in MATLAB. This is easier to build/run due to issues with ROS2 support in MATLAB, but decreases performance.

## Subscriptions
- `avt_341/odometry` ([`nav_msgs/Odometry`](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html))
- `avt_341/points` ([`sensor_msgs/PointCloud2`](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))
- `camera/rgb/image_raw` ([`sensor_msgs/Image`](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

## Publishers
- `avt_341/segmentation_grid` ([`nav_msgs/OccupancyGrid`](http://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

## Before running:
- Install MATLAB Runtime 2023a
- Update `Matlab_MCLMCRRT_LIB` path in `CMakeLists.txt` to MATLAB Runtime install location
- Download the DLL from [here](https://drive.google.com/file/d/1j6TEM9lfAfgaeCVbMfLkTCo0M9c7FW8X/view?usp=sharing) into the `uab_perception` folder

## Building
- `colcon build --packages-select avt_341`
- `call install/setup.bat`

## Running
Run the UAB perception node with the rest of the NATO AVT-341 stack with:
- `ros2 launch avt_341 krc_uab_segmentation.launch.py`

## Troubleshooting
"Fatal error C1083: Cannot open include file: 'mclmcrrt.h': No such file or directory"
  - Set the MATLAB include directory manually under the `Matlab_MCLMRRT_LIB` line in the CMakeLists file.
    - `set(Matlab_INCLUDE_DIRS "path/to/MATLAB Runtime/v912/extern/include")`
NATO perception node occupancy grid not being drawn/out of sync
  - There is an oustanding issue with the timestamps between the odometry and pointcloud. You can solve this by temporarily replacing the `GetPoseToUse` function in `avt_341_perception_node.cpp` with
    ```cpp
    double GetPoseToUse(avt_341::msg::Odometry & pose_to_use, avt_341::msg::PointCloud2Ptr rcv_cloud){
      double dt = 1.0;
      for (int i=0;i<current_pose_list.size();i++){
        pose_to_use = current_pose_list[i];
      }
      return dt;
    }
    ```