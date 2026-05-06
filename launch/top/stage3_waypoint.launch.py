"""
stage3_waypoint.launch.py
======================================================================
3단계: Gazebo + SLAM + Nav2 + AprilTag + Waypoint 자율 탐사 + mine pipeline

Stage 2 에서 수동으로 /finish_exploration 을 발행하던 것을
waypoint_follower_node 가 자동으로 수행한다.

동작 순서:
  1) Jackal Gazebo (mine_detection.world)
  2) [5s 후] SLAM toolbox
  3) [8s 후] Nav2
  4) [5s 후] AprilTag 인식
  5) mine_recorder_node
  6) mine_cluster_node + exploration_manager_node + mine_goal_sender_node
  7) [12s 후] waypoint_follower_node (Nav2 완전 기동 후 시작)
     → params.yaml 의 waypoints 순서대로 이동
     → 모두 완료 시 /finish_exploration=True 자동 발행
     → exploration_manager → /trigger_final_goal
     → mine_goal_sender → Nav2 NavigateToPose goal 전송

성공 기준:
  - Jackal 이 waypoint 를 순서대로 이동하면서 AprilTag 를 인식
  - 탐사 완료 후 자동으로 지뢰 밀집 구역으로 이동하여 정지
======================================================================
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction, SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _try_pkg(name: str):
    try:
        return get_package_share_directory(name)
    except Exception:
        return None


def generate_launch_description():
    pkg_share = get_package_share_directory('jackal_mine_detection')
    actions = []

    # ── GAZEBO_MODEL_PATH ─────────────────────────────────────────────
    models_path = os.path.join(pkg_share, 'models')
    actions.append(
        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', ''),
        )
    )

    # ── 1) Jackal Gazebo ──────────────────────────────────────────────
    world_file = os.path.join(pkg_share, 'worlds', 'mine_detection.world')
    jackal_gazebo_share = _try_pkg('jackal_gazebo')
    if jackal_gazebo_share is not None:
        for fname in ['jackal_world.launch.py', 'gazebo.launch.py', 'empty_world.launch.py']:
            p = os.path.join(jackal_gazebo_share, 'launch', fname)
            if os.path.exists(p):
                actions.append(IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(p),
                    launch_arguments={'world': world_file}.items(),
                ))
                break
    else:
        print('[stage3] WARNING: jackal_gazebo 패키지를 찾을 수 없습니다.')

    # ── 2) SLAM [5s] ──────────────────────────────────────────────────
    actions.append(TimerAction(period=5.0, actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sensor', 'slam.launch.py')
        ))
    ]))

    # ── 3) Nav2 [8s] ─────────────────────────────────────────────────
    actions.append(TimerAction(period=8.0, actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sensor', 'nav2.launch.py')
        ))
    ]))

    # ── 4) AprilTag [5s] ─────────────────────────────────────────────
    actions.append(TimerAction(period=5.0, actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sensor', 'apriltag.launch.py')
        ))
    ]))

    # ── 5) mine_recorder_node ─────────────────────────────────────────
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pkg_share, 'launch', 'mission', 'apriltag_recorder.launch.py')
    )))

    # ── 6) mine pipeline ─────────────────────────────────────────────
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pkg_share, 'launch', 'mission', 'mine_pipeline.launch.py')
    )))

    # ── 7) Waypoint explorer [12s 후] (Nav2 완전 기동 대기) ───────────
    actions.append(TimerAction(period=12.0, actions=[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'mission', 'waypoint_explorer.launch.py')
        ))
    ]))

    return LaunchDescription(actions)
