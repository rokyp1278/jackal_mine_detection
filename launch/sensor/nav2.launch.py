"""
nav2.launch.py
Nav2 navigation stack 실행 (slam_toolbox 의 map 을 사용).
nav2_params.yaml 이 로봇/환경 설정을 담고 있다.

사용:
  ros2 launch jackal_mine_detection nav2.launch.py
  ros2 launch jackal_mine_detection nav2.launch.py use_sim_time:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share   = get_package_share_directory('jackal_mine_detection')
    nav2_share  = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    nav2_launch = os.path.join(nav2_share, 'launch', 'navigation_launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='시뮬레이션 시간 사용 여부',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file':  nav2_params,
            }.items(),
        ),
    ])
