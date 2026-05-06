"""
stage2_sim.launch.py
======================================================================
Terminal 1 전용: Gazebo + 로봇 스폰만 담당.
이 launch 를 실행하고 Gazebo 창에 노란 로봇이 나타나면,
Terminal 2 에서 stage2_nav.launch.py 를 실행하세요.

사용법:
  T1: ros2 launch jackal_mine_detection stage2_sim.launch.py
  T2: (Gazebo 로봇 확인 후) ros2 launch jackal_mine_detection stage2_nav.launch.py
======================================================================
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('jackal_mine_detection')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'sensor', 'gazebo.launch.py')
            )
        )
    ])
