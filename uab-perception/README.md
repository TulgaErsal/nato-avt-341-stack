# UAB Perception ROS2 Package
### Performs semantic segmentation on camera images using the DeepLab V3+ architecture and fuses with LiDAR pointcloud data to create a costmap based on vehicle traversability. Requires MATLAB 2022a.
---
There are currently two ways to run the UAB Perception package:
1. Run using the `perception_sem_seg` ROS2 node: `ros2 run avt_341 perception_sem_seg`
1. Run the `ex_PerceptionAlgorithm.m` file directly in MATLAB 2022a

The `perception_sem_seg` node is a Python wrapper for the machine learning model written in MATLAB. This is easier to build/run due to issues with ROS2 support in MATLAB, but decreases performance.

Before running, set `matlab_code_folder` in `perception_sem_seg.py` to full path of the `uab-perception/semantic_segmentation` directory.

## Subscriptions
- `avt_341/odometry` ([`nav_msgs/Odometry`](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html))
- `avt_341/points` ([`sensor_msgs/PointCloud2`](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))
- `camera/rgb/image_raw` ([`sensor_msgs/Image`](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

## Publishers
- `avt_341/occupancy_grid` ([`nav_msgs/OccupancyGrid`](http://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))
