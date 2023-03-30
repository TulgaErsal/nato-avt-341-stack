# Costmap Clearing 

__Contents__: 
- [Configuration](#configuration)
- [Raytrace Clearing - LiDAR TF Transform](#ray-tracing-lidar-tf-transform)
- [Raytrace Clearing - Voxel Representation](#raytrace-clearing-voxel-representation)
- [Sample Launch File Parameters](#sample-launch-file-parameters)

## Configuration

- The costmap clearing method can be set using the `clear_method` parameter.
- See additional parameters in `base.launch` / `base.launch.py` file.

| Clear Method Key    | Description                                                                                                                                                                                                         |
|---------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `none` (default)      | No costmap clearing. Costmap cell values persist forever.                                                                                                                                                           |   
| `time`                | Unchanged costmap cells cleared every `max_point_age`.                                                                                                                                                              |   
| `raytrace`            | Lidar rays that pass through previously occupied cells will clear them.                                                                                                                                             |   
| `raytrace_obs_filter` | Same as `raytrace` option but also prevents clearing of cells that are within `clear_method_obs_filter_range` of obstacles. Used to reduce intermittent clearing of edge cells that share obstacles and free space. |

## Raytrace Clearing LiDAR TF Transform
- The raytracing requires a `map` to `lidar` transform to find the origin for ray tracing.

## Raytrace Clearing Voxel Representation
- The raytrace clearing method also discretizes the z-axis into a number of voxels per cell.
- If raytrace clearing is used, the `clear_method_voxel_height_min` and `clear_method_voxel_height_res` must be specified.
- See sample launch file below for an example.

![Voxel Representation](https://github.com/stefanwapnick/avt_341_resources/raw/master/OccupancyVoxelGrid.png)

## Sample Launch File Parameters
- Below are sample launch file parameters for ros2 and raytrace clearing. 
- These parameters override the default `base.launch.py` parameters.

```
 avt_341_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(avt_341_dir, 'launch', 'dwa_base.launch.py')),
        launch_arguments={'robot_description': robot_desc,
                          'grid_llx': '-420.0',
                          'grid_lly': '-780.0',
                          'grid_width': '800.0', 
                          'grid_height': '1600.0',
                          'grid_res': '1.0',
                          'grid_dilate': 'True',
                          'grid_dilate_x': '1.0',
                          'grid_dilate_y': '1.0',
                          'max_point_age': '1000.0',
                          'clear_method': 'raytrace',
                          'clear_method_voxel_height_min': '150.0',
                          'clear_method_voxel_height_res': '0.2',
                          'clear_method_immediate_clear_dilation': 'True',
                          'clear_method_obs_filter_range': '1.0',
                          'slope_threshold': '0.4',
                          'path_look_ahead': '15.0',
                          'vehicle_speed': '8.0',
                          'goal_dist': '2.0',
                          'vehicle_width': '1.6',
                          'max_steer_angle': '0.8',
                          'max_desired_lateral_g': '3000.0',
                          'shutdown_behavior': '1',
                          'dwa_speed_lin_min': '4.0',
                          'dwa_speed_lin_max': '10.0',
                          'dwa_accel_max': '4.0',
                          'dwa_speed_ang_min': '-0.785',
                          'dwa_speed_ang_max': '0.785',
                          'dwa_ang_accel_max': '10.0',
                          'dwa_time_span_min': '4.2',
                          'dwa_w_cost_dev': '0.2',
                          'dwa_use_global_path': 'True',
                          'dwa_w_cost_path': '0.4',
                          'dwa_collision_radius': '1.5',
                          'vehicle_max_steer_angle_degrees': '45.0',
                          'los_max_iterations': '5',
                          'los_break_on_first': 'False',
                          'search_diagonals': 'True'
                          }.items(),
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('use_mpc'))
    )
```

