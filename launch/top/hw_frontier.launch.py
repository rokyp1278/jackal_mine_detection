"""
hw_frontier.launch.py
======================================================================
실제 Jackal 하드웨어 전용: SLAM + Nav2 + RealSense + AprilTag 탐지
+ mine pipeline + frontier 자율 탐사.

stage3_frontier.launch.py 의 실제 장비 버전.
  - Gazebo 없음
  - use_sim_time: False
  - sim_mine_detector 대신 RealSense + AprilTag + mine_recorder 사용

[사전 조건]
  sudo apt install ros-humble-realsense2-camera
  ros-humble-apriltag-ros 패키지 설치 확인

[실행 방법]
  Terminal 1 (Jackal base driver — Jackal 패키지 별도 설치 필요):
    ros2 launch jackal_robot jackal_robot.launch.py

  Terminal 2 (이 파일):
    cd ~/ros2_ws
    colcon build --packages-select jackal_mine_detection --symlink-install
    source install/setup.bash
    ros2 launch jackal_mine_detection hw_frontier.launch.py

[동작 순서]
  0s  : RealSense 카메라 + AprilTag 탐지 + mine pipeline 시작
  5s  : SLAM 시작
  15s : Nav2 시작
  50s : frontier_explorer_node 시작 (내부 start_delay 5s 포함)
======================================================================
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share   = get_package_share_directory('jackal_mine_detection')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')
    launch_dir  = os.path.join(pkg_share, 'launch')

    def _inc(subdir, name, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, subdir, name)
            ),
            launch_arguments=(args or {}).items(),
        )

    # ── mine_cluster / exploration_manager / mine_goal_sender ─────────
    # mine_pipeline.launch.py 의 params.yaml 은 use_sim_time:true 고정
    # → 하드웨어에서는 노드를 직접 기동해 override
    mine_cluster = Node(
        package='jackal_mine_detection',
        executable='mine_cluster_node',
        name='mine_cluster_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': False}],
    )
    exploration_manager = Node(
        package='jackal_mine_detection',
        executable='exploration_manager_node',
        name='exploration_manager_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': False}],
    )
    mine_goal_sender = Node(
        package='jackal_mine_detection',
        executable='mine_goal_sender_node',
        name='mine_goal_sender_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': False}],
    )

    # ── AprilTag → map 좌표 저장 (실제 카메라 frame 사용) ─────────────
    mine_recorder = Node(
        package='jackal_mine_detection',
        executable='mine_recorder_node',
        name='mine_recorder_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time':  False,
            'camera_frame':  'camera_color_optical_frame',  # RealSense 기본 frame
            'map_frame':     'map',
        }],
    )

    # ── Frontier 자율 탐사 ─────────────────────────────────────────────
    frontier_explorer = Node(
        package='jackal_mine_detection',
        executable='frontier_explorer_node',
        name='frontier_explorer_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time':    False,
            'start_delay_sec': 5.0,
        }],
    )

    return LaunchDescription([
        # ── 즉시 시작 ─────────────────────────────────────────────────
        _inc('sensor', 'realsense.launch.py'),
        _inc('sensor', 'apriltag.launch.py', {'use_sim_time': 'false'}),
        mine_recorder,
        mine_cluster,
        exploration_manager,
        mine_goal_sender,

        # ── SLAM [5s 후] ─────────────────────────────────────────────
        TimerAction(
            period=5.0,
            actions=[_inc('sensor', 'slam.launch.py', {'use_sim_time': 'false'})],
        ),

        # ── Nav2 [15s 후] ────────────────────────────────────────────
        TimerAction(
            period=15.0,
            actions=[_inc('sensor', 'nav2.launch.py', {'use_sim_time': 'false'})],
        ),

        # ── Frontier 탐사 [50s 후] ───────────────────────────────────
        TimerAction(period=50.0, actions=[frontier_explorer]),
    ])
