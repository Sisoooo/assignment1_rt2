from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess


def generate_launch_description():
    container = ComposableNodeContainer(
        name='movement_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='client_server_cpp',
                plugin='client_server_cpp::MovementServer',
                name='movement_server'
            ),
            ComposableNode(
                package='client_server_cpp',
                plugin='client_server_cpp::MovementClient',
                name='movement_client'
            ),
        ],
    )

    movement_ui = ExecuteProcess(
        cmd=[
            'xterm', '-hold', '-e',
            'ros2', 'run', 'client_server_cpp', 'movement_ui',
        ],
        output='screen',
    )

    return LaunchDescription([container, movement_ui])
