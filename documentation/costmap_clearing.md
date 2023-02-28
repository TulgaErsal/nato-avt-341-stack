# Costmap Clearing 

__Contents__: 
- [Configuration](#configuration)
- [Debugging Tools](#debugging-tools)
- [Use Dilation When Raytrace Clearing Selected](#use-dilation-when-raytrace-clearing-selected)

## Configuration

This page describes the available costmap clearing methods. The costmap clearing method can be set using the `clear_method` parameter.

Example:
```bash
roslaunch avt_341 example.launch clear_method:=<method>
```

| Method         | Description                                                                                                                                    |
|----------------|------------------------------------------------------------------------------------------------------------------------------------------------|
| none           | No costmap clearing. Costmap cell values persist forever.                                                                                      |   
| time (default) | Costmap cells cleared every `max_point_age`.                                                                                                   |   
| raytrace       | Lidar rays that pass through previously occupied cells will clear them.                                                                        |   
| raytrace_voxel | Lidar rays that pass through previously occupied cells will clear them. Also uses voxel grid to determine next max-min values during clearing. |

## Debugging Tools
When `raytrace` or `raytrace_voxel` are selected, set `clear_method_visualize=true` to obtain additional debug information in rviz via topic `avt_341/costmap/voxels`.

![RVIZ Voxel Grid](https://github.com/stefanwapnick/avt_341_resources/raw/master/OccupancyVoxelGrid.png)  
![#1589F0](https://placehold.co/15x15/1589F0/1589F0.png) Min cell value
![#f03c15](https://placehold.co/15x15/f03c15/f03c15.png) Max cell value
![#aaaaaa](https://placehold.co/15x15/aaaaaa/aaaaaa.png) Intermediate voxel Cells 

## Use Dilation When Raytrace Clearing Selected
TODO