import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define file paths
    pkg_name = 'avt_341'
    default_urdf_path = os.path.join(get_package_share_directory(pkg_name), 'config', 'MRZR.urdf')
    default_calib_path = os.path.join(get_package_share_directory(pkg_name), 'config', 'sensor_calibration', '2025-03-18-13-24-22-camera-info-matlab.yaml')

    # Load URDF file as a parameter
    with open(default_urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    return launch.LaunchDescription([
        # Start robot_state_publisher with URDF
        # launch_ros.actions.Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     parameters=[{'robot_description': robot_description_content}]
        # ),

        # Start camera-lidar TF publisher with calibration file
        launch_ros.actions.Node(
            package=pkg_name,
            executable='calibration_tf_publisher_node',
            name='calibration_tf_publisher_node',
            output='screen',
            parameters=[{'calibration_file': default_calib_path,
                         'lidar_frame': 'os_sensor',
                         'camera_frame': 'flir_optical'}]
        ),
    ])