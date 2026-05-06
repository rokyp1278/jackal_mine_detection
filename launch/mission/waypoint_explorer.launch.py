"""
waypoint_explorer.launch.py
waypoint_follower_node 실행 (3단계 이상).
mine_pipeline.launch.py 와 함께 사용한다.

사용:
  ros2 launch jackal_mine_detection waypoint_explorer.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share   = get_package_share_directory('jackal_mine_detection')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='jackal_mine_detection',
            executable='waypoint_follower_node',
            name='waypoint_follower_node',
            output='screen',
            parameters=[params_file, {'use_sim_time': True}],
        ),
    ])
