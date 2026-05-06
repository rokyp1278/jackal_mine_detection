"""
mine_pipeline.launch.py
mine_cluster_node + exploration_manager_node + mine_goal_sender_node 를 같이 실행.
fake_mine 또는 AprilTag 기반 mine_recorder 와 조합되어 사용된다.
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
            executable='mine_cluster_node',
            name='mine_cluster_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='jackal_mine_detection',
            executable='exploration_manager_node',
            name='exploration_manager_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='jackal_mine_detection',
            executable='mine_goal_sender_node',
            name='mine_goal_sender_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
