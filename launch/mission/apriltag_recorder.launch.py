"""
apriltag_recorder.launch.py
mine_recorder_node 실행 (2단계 이상).
apriltag.launch.py 와 함께 사용한다.

사용:
  ros2 launch jackal_mine_detection apriltag_recorder.launch.py
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
            executable='mine_recorder_node',
            name='mine_recorder_node',
            output='screen',
            # camera_frame 을 map 으로 설정 →
            # sim_apriltag_detector 가 map 좌표로 발행하므로 map→map 변환 = 무변환
            parameters=[params_file, {
                'use_sim_time':  True,
                'camera_frame':  'map',
                'map_frame':     'map',
            }],
        ),
    ])
