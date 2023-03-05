# Costmap Clearing 

__Contents__: 
- [Configuration](#configuration)
- [LiDAR TF Transform for Ray Tracing](#lidar-tf-transform-for-ray-tracing)

## Configuration

- The costmap clearing method can be set using the `clear_method` parameter.

Example:
```bash
roslaunch avt_341 <launch_file> clear_method:=<method>
ros2 launch avt_341 <launch_File> clear_method:=<method>
```

- See additional parameters in `base.launch` / `base.launch.py` file.

| Method              | Description                                                                                                                                                                                                         |
|---------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| none (default)      | No costmap clearing. Costmap cell values persist forever.                                                                                                                                                           |   
| time                | Unchanged costmap cells cleared every `max_point_age`.                                                                                                                                                              |   
| raytrace            | Lidar rays that pass through previously occupied cells will clear them.                                                                                                                                             |   
| raytrace_obs_filter | Same as `raytrace` option but also prevents clearing of cells that are within `clear_method_obs_filter_range` of obstacles. Used to reduce intermittent clearing of edge cells that share obstacles and free space. |

## LiDAR TF Transform for Ray Tracing
- The raytracing code looks up the `map` to `lidar` transform to find the origin for ray tracing.