import os

import launch_ros
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch.conditions
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():

    project = 'avt_341'
    config = 'ouster_krc'
    #config = 'dummy'
    use_sim_time=True

    avt_341_package_dir = get_package_share_directory(project)
    avt_341_dir = get_package_share_directory(project)
    print(avt_341_dir,avt_341_package_dir)
    
    paramspath = os.path.join(avt_341_dir,'launch','slam','config','params_'+config+'.yaml')
    with open(paramspath, 'r') as f:
        params = yaml.safe_load(f)
    
    params2 = {}
    for k,v in params['liorf'].items():
        params2['liorf/'+k] = v
    params2['use_sim_time'] = use_sim_time    
    for k,v in params2.items():
        print(k,":",v)

    image_projection_node = launch_ros.actions.Node(
            package=project,
            #namespace='',
            executable=project+'_slam_image_projection_node',
            name=project+'_slam_image_projection',
            parameters=[params2],
            #remappings=[
            #],
            output='screen',
        )

    imu_preintegration_node = launch_ros.actions.Node(
            package=project,
            #namespace='',
            executable=project+'_slam_imu_preintegration_node',
            name=project+'_slam_imu_preintegration',
            parameters=[params2],
            #remappings=[
            #],
            output='log',
        )

    map_optimization_node = launch_ros.actions.Node(
            package=project,
            #namespace='',
            executable=project+'_slam_map_optimization_node',
            name=project+'_slam_map_optimization',
            parameters=[params2],
            #remappings=[
            #],
            output='log',
        )


    launch_description = LaunchDescription([
        image_projection_node,
        imu_preintegration_node,
        map_optimization_node,
    ])

    return launch_description
