# LIDAR Occlusion

## Requirements
This requires MAVS, the MAVS ROS package (https://mavs-documentation.readthedocs.io/en/latest/Interfaces/MavsROS/) and AVT_341 autonomy stack. 

## Running the lidar occlusion scenes. 

Main launch file: 'lidar_occlusion.launch'
'lidar_occlusion.launch' references 'mavs_lidar_occlusion_base.launch'
'mavs_lidar_occlusion_base.launch' is based on the 'mavs_example.launch' file

* scene_file_arg - specify which scene file MAVS should load
* waypoints_file - specify waypoints to use; occlusion_scene_maze should use waypoints_maze.yaml
* Occlusion Parameters - see Lidar Occlusion Node notes below

'roslaunch avt_341 lidar_occlusion.launch' will launch MAVS and avt_341 with the specified scene, waypoints, and occlusion parameters

### Scene Files
Five scene files have been created for the lidar occlusion project. 
* occlusion_scene_odoa.json
* occlusion_scene_random_field.json
* occlusion_scene_slalom.json
* occlusion_scene_three_obstacles.json
* occlusion_scene_maze.json

# Nodes

## AVT 341 Lidar Occlusion Node
avt_341_lidar_occlusion_node.cpp

The LiDAR Occlusion node subscribes to sensor point cloud data (published on avt_341/points) and applies a rectangular mask (specified with parameters) and publishes the occluded point cloud data (on avt_341/occ_points). 

### Occlusion Parameters
Occlusions are currently rectangular. The mask is simply an int vector so more complex masks are possible but the generator is a bit buggy so the current version just exposes a simple rectangular mask. Mask of 2,384, 60, 256 blocks the front quadrant of the vehicle's LIDAR. 

* occluded_lidar - true if the lidar should be occluded, false if not
* occlusion_timer - time in seconds from first point cloud received by the stack to applying the mask
* occlusion_start_row - initial row (height) of lidar to apply occlusion
* occlusion_start_col - initial column (angle) of lidar to apply occlusion
* occlusion_mask_height - height of the occlusion
* occlusion_mask_width - width of the occlusion



## AVT 341 Lidar Occlusion Detection Node
avt_341_lidar_occlusion_detection_node.cpp

This node subscribes to sensor point cloud data (typically avt_341/points; remapped to avt_341/occ_points when using the lidar occlusion node), processes the PCD, and publishes a mask representing detected occlusions. 


