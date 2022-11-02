# UAB Perception ROS2 Node
## Performs semantic segmentation on camera images using the DeepLab V3+ architecture and fuses with LiDAR pointcloud data to create a costmap based on vehicle traversability. Requires MATLAB 2022a Runtime.

The `uab_perception_node` is a C++ wrapper for the machine learning model written in MATLAB. This is easier to build/run due to issues with ROS2 support in MATLAB, but decreases performance.

## Subscriptions
- `avt_341/odometry` ([`nav_msgs/Odometry`](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html))
- `avt_341/points` ([`sensor_msgs/PointCloud2`](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html))
- `camera/rgb/image_raw` ([`sensor_msgs/Image`](http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html))

## Publishers
- `avt_341/segmentation_grid` ([`nav_msgs/OccupancyGrid`](http://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))
- `avt_341/segmentation_grid_vis` ([`nav_msgs/OccupancyGrid`](http://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

## Before running:
- Install MATLAB Runtime 2022a
- Update `Matlab_MCLMCRRT_LIB` path in `CMakeLists.txt` to MATLAB Runtime install location

## Building
- `colcon build --packages-select avt_341`
- `call install/setup.bat`

## Running
- Run the UAB Perception node (currently two options):
  1. Launch the UAB perception node with the rest of the NATO AVT-341 stack: `ros2 launch avt_341 single_tile.launch.py`
  1. Or run the `ex_PerceptionAlgorithm.m` file directly in MATLAB 2022a and launch the rest of the NATO stack.
