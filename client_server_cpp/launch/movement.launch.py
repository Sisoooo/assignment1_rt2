from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    movement_server = Node(
        package='client_server_cpp',
        executable='movement_server',
        name='movement_server',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    movement_ui = ExecuteProcess(
        cmd=[
            'xterm', '-hold', '-e',
            'ros2', 'run', 'client_server_cpp', 'movement_ui',
        ],
        output='screen',
    )

    return LaunchDescription([
        movement_server,
        movement_ui,
    ])
