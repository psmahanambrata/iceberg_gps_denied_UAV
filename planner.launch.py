from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_denied_navigation',
            executable='astar_planner',
            name='astar_planner',
            output='screen',
            parameters=[{'occupied_threshold': 50, 'allow_unknown': False}],
        ),
    ])
