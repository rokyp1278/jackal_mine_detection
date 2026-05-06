"""
fake_mine.launch.py
fake_mine_publisher_node 만 실행. 1단계용.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('jackal_mine_detection')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='jackal_mine_detection',
            executable='fake_mine_publisher_node',
            name='fake_mine_publisher_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
