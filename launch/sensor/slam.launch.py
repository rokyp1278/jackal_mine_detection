"""
slam.launch.py
slam_toolbox online_async 모드로 SLAM 실행.
slam_params.yaml 을 사용하므로 별도 인자 없이 바로 실행 가능.

사용:
  ros2 launch jackal_mine_detection slam.launch.py
  ros2 launch jackal_mine_detection slam.launch.py use_sim_time:=false  (실제 HW)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share  = get_package_share_directory('jackal_mine_detection')
    slam_share = get_package_share_directory('slam_toolbox')

    slam_params = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    slam_launch = os.path.join(slam_share, 'launch', 'online_async_launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='시뮬레이션 시간 사용 여부 (Gazebo=true, 실제HW=false)',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'use_sim_time':      use_sim_time,
                'slam_params_file':  slam_params,
            }.items(),
        ),
    ])
