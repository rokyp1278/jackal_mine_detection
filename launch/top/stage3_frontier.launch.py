"""
stage3_frontier.launch.py
======================================================================
Terminal 2 전용: SLAM + Nav2 + 탐지 + mine pipeline + frontier 자율 탐사.
반드시 stage2_sim.launch.py 로 Gazebo + 로봇이 먼저 실행된 상태에서 시작.

stage2_nav.launch.py 와 동일한 구성이지만
waypoint_follower_node → frontier_explorer_node 로 교체.

[동작 순서]
  0s  : mine_pipeline + sim_mine_detector 시작
  5s  : SLAM 시작
  15s : Nav2 시작
  50s : frontier_explorer_node 시작 (내부 start_delay 5s 포함)

[frontier 탐사 흐름]
  /map 수신 → frontier(free-unknown 경계) BFS 탐지
  → 가장 유망한 frontier로 NavigateToPose
  → 반복 → frontier 소진 시 /finish_exploration=True
  → exploration_manager → /trigger_final_goal
  → mine_goal_sender → 지뢰 군집 중심으로 이동
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

    # TF 기반 지뢰 탐지기 (카메라/AprilTag 우회)
    sim_detector = Node(
        package='jackal_mine_detection',
        executable='sim_mine_detector_node',
        name='sim_mine_detector_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'detect_range_m': 2.0,
            'fov_half_deg':  180.0,  # 시뮬레이션: FOV 제한 없음 (거리만 체크)
            # 실제 하드웨어에서는 RealSense 카메라가 자체 FOV 처리
        }],
    )

    # Frontier 자율 탐사 노드
    frontier_explorer = Node(
        package='jackal_mine_detection',
        executable='frontier_explorer_node',
        name='frontier_explorer_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': True,
            'start_delay_sec': 5.0,
        }],
    )

    return LaunchDescription([
        # ── 즉시 시작 ─────────────────────────────────────────────────
        _inc('mission', 'mine_pipeline.launch.py'),
        sim_detector,

        # ── SLAM [5s 후] ─────────────────────────────────────────────
        TimerAction(period=5.0,  actions=[_inc('sensor', 'slam.launch.py')]),

        # ── Nav2 [15s 후] ─────────────────────────────────────────────
        TimerAction(period=15.0, actions=[_inc('sensor', 'nav2.launch.py')]),

        # ── Frontier 탐사 [50s 후] ───────────────────────────────────
        TimerAction(period=50.0, actions=[frontier_explorer]),
    ])
