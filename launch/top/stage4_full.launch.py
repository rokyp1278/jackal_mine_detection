"""
stage4_full.launch.py
======================================================================
Terminal 2 전용: SLAM + Nav2 + TF 탐지(3m) + frontier 자율 탐사.
반드시 stage2_sim.launch.py 로 Gazebo + 로봇이 먼저 실행된 상태에서 시작.

stage3_frontier.launch.py 와의 차이:
  detect_range_m 2.0 → 3.0 (시작 위치에서 태그까지 2.09m라 2m로는 탐지 불가)
  sim_mine_detector → frontier_explorer (완전 자율 탐사)

[동작 순서]
  0s  : sim_mine_detector + mine_pipeline 시작
  5s  : SLAM 시작
  15s : Nav2 시작
  50s : frontier_explorer_node 시작 (내부 start_delay 5s 포함)

[데이터 흐름]
  sim_mine_detector → /mine_positions
    → mine_cluster_node → /mine_cluster_center
  /map → frontier_explorer_node → NavigateToPose
    → /finish_exploration → exploration_manager_node → /trigger_final_goal
      → mine_goal_sender_node → 지뢰 군집 중심 이동
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

    def _inc(subdir, name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, subdir, name)
            )
        )

    # TF 기반 지뢰 탐지기 — detect_range 3.0m (시작위치→태그 최단거리 2.09m)
    sim_detector = Node(
        package='jackal_mine_detection',
        executable='sim_mine_detector_node',
        name='sim_mine_detector_node',
        output='screen',
        parameters=[{
            'use_sim_time':   True,
            'detect_range_m': 3.0,
            'fov_half_deg':   180.0,
        }],
    )

    # Frontier 자율 탐사 노드
    frontier_explorer = Node(
        package='jackal_mine_detection',
        executable='frontier_explorer_node',
        name='frontier_explorer_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time':    True,
            'start_delay_sec': 5.0,
        }],
    )

    return LaunchDescription([
        # ── 즉시 시작 ─────────────────────────────────────────────────
        sim_detector,
        _inc('mission', 'mine_pipeline.launch.py'),

        # ── SLAM [5s 후] ─────────────────────────────────────────────
        TimerAction(period=5.0,  actions=[_inc('sensor', 'slam.launch.py')]),

        # ── Nav2 [15s 후] ─────────────────────────────────────────────
        TimerAction(period=15.0, actions=[_inc('sensor', 'nav2.launch.py')]),

        # ── Frontier 탐사 [50s 후] ───────────────────────────────────
        TimerAction(period=50.0, actions=[frontier_explorer]),
    ])
