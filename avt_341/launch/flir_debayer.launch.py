from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::DebayerNode',
            name='debayer_node',
            namespace='flir_camera'
        )
    ]

    container = ComposableNodeContainer(
        name='image_proc_container',
        package='rclcpp_components',
        executable='component_container',
        namespace='flir_camera',
        composable_node_descriptions=composable_nodes,
    )

    return LaunchDescription([container])