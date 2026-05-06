"""
stage2_apriltag.launch.py
======================================================================
2단계: Gazebo + SLAM + Nav2 + AprilTag 인식 + mine pipeline
자체 URDF(jackal_sim.urdf) 사용 → jackal_gazebo 패키지 불필요.

실행 방법 (터미널 3개):
  T1: ros2 launch jackal_mine_detection stage2_apriltag.launch.py
  T2: rviz2 -d $(ros2 pkg prefix jackal_mine_detection)/share/jackal_mine_detection/config/rviz/mine_detection.rviz
  T3: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel

탐사 종료 신호 (T4):
  ros2 topic pub --once /finish_exploration std_msgs/msg/Bool '{data: true}'

성공 기준:
  - Gazebo 에 노란 로봇 + 복도 + AprilTag 표시
  - RViz /mine_markers 에 주황 큐브 표시 (태그 감지 시)
  - /finish_exploration 후 Nav2 goal 전송
======================================================================
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share  = get_package_share_directory('jackal_mine_detection')
    launch_dir = os.path.join(pkg_share, 'launch')

    def _inc(subdir, name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, subdir, name)
            )
        )

    return LaunchDescription([
        # 1) Gazebo + robot (즉시)
        _inc('sensor', 'gazebo.launch.py'),

        # 2) SLAM [5s 후] (Gazebo + LiDAR 준비 대기)
        TimerAction(period=5.0,  actions=[_inc('sensor', 'slam.launch.py')]),

        # 3) Nav2 [13s 후] (SLAM map 최초 퍼블리시 + TF 안정화 대기)
        TimerAction(period=13.0, actions=[_inc('sensor', 'nav2.launch.py')]),

        # 4) AprilTag [6s 후] (카메라 토픽 준비 대기)
        TimerAction(period=6.0,  actions=[_inc('sensor', 'apriltag.launch.py')]),

        # 5) mine_recorder (즉시, 수신 데이터 기다림)
        _inc('mission', 'apriltag_recorder.launch.py'),

        # 6) mine pipeline (즉시, 수신 데이터 기다림)
        _inc('mission', 'mine_pipeline.launch.py'),
    ])
