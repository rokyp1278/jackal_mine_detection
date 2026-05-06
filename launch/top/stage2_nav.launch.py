"""
stage2_nav.launch.py
======================================================================
Terminal 2 전용: SLAM + Nav2 + 탐지 + mine pipeline + waypoint 자율 탐사.
반드시 stage2_sim.launch.py 로 Gazebo + 로봇이 먼저 실행된 상태에서 시작.

[동작 순서]
  0s  : mine_pipeline (cluster + manager + goal_sender) + sim_mine_detector 시작
  5s  : SLAM 시작
  15s : Nav2 시작
  35s : waypoint_follower 시작 (Nav2 완전 기동 대기 후 자율 탐사 개시)

[WSL 카메라 대체]
  sim_mine_detector_node 가 TF 기반으로 지뢰 탐지를 시뮬레이션.
  카메라/AprilTag 파이프라인 완전 우회.
======================================================================
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share  = get_package_share_directory('jackal_mine_detection')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')
    launch_dir = os.path.join(pkg_share, 'launch')

    def _inc(subdir, name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, subdir, name)
            )
        )

    # TF 기반 지뢰 탐지기 (카메라/AprilTag 파이프라인 완전 우회)
    sim_detector = Node(
        package='jackal_mine_detection',
        executable='sim_mine_detector_node',
        name='sim_mine_detector_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'detect_range_m': 2.0,
            'fov_half_deg':  45.0,
        }],
    )

    # Waypoint 자율 탐사 노드 (Nav2 기동 후 시작)
    # start_delay_sec=20.0: Nav2(15s) + 초기화(5s) 대기 후 탐사 시작
    waypoint_follower = Node(
        package='jackal_mine_detection',
        executable='waypoint_follower_node',
        name='waypoint_follower_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': True,
            'start_delay_sec': 20.0,
        }],
    )

    return LaunchDescription([
        # ── 즉시 시작 ─────────────────────────────────────────────────
        _inc('mission', 'mine_pipeline.launch.py'),   # cluster + manager + goal_sender
        sim_detector,                                  # TF 기반 탐지기

        # ── SLAM [5s 후] ─────────────────────────────────────────────
        TimerAction(period=5.0,  actions=[_inc('sensor', 'slam.launch.py')]),

        # ── Nav2 [15s 후] ─────────────────────────────────────────────
        TimerAction(period=15.0, actions=[_inc('sensor', 'nav2.launch.py')]),

        # ── Waypoint 자율 탐사 [50s 후] ──────────────────────────────
        # Nav2(15s) 완전 활성화까지 35s 필요 → 50s에 노드 시작 후 start_delay 5s
        TimerAction(period=50.0, actions=[waypoint_follower]),
    ])
