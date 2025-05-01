from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

GLOBAL_FRAME = 'map'
TRACKER_CONFIG = "ekf_tracker_krc"
CFG_PATH = os.path.dirname(os.path.abspath(__file__))+"/../cfg/%s.yaml" % (TRACKER_CONFIG)
print(CFG_PATH)

USE_SIM_TIME = True

def generate_launch_description():


    ekf_tracker_launch = LaunchDescription([
        Node(
            package='ekf3d',
            executable='online_tracker',
            name='tracker',
            output='screen',
            parameters=[{
                'cfg_path' : CFG_PATH,
                'frame_id' : GLOBAL_FRAME,
                'update_freq' : 3.0,
                'use_sim_time' : USE_SIM_TIME,
                'update_from_lidar' : True,
                'update_from_yolo' : False,
                'update_from_tick' : False,
                }],
            remappings=[
                ('detected_bbx', '/avt408/lidar_detector/detected_bbx/in_%s' % GLOBAL_FRAME),
                ('detection_centroids', '/avt408/cameralidar_detector/detection_centroids/in_%s' % GLOBAL_FRAME),
                ('odom', '/taros/odom')
                ],
            )
    ])



    return ekf_tracker_launch
