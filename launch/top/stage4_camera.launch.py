"""
stage4_camera.launch.py
======================================================================
Terminal 2 전용: SLAM + Nav2 + Gazebo 카메라 실제 AprilTag 인식
+ mine pipeline + frontier 자율 탐사.
반드시 stage2_sim.launch.py 로 Gazebo + 로봇이 먼저 실행된 상태에서 시작.

stage4_full.launch.py 와의 차이:
  sim_apriltag_detector (TF 기반 가짜 탐지)
  → apriltag_ros (실제 카메라 이미지 기반 탐지)
  WSL + OGRE_RTT_MODE=Copy 환경에서 카메라 토픽이 정상 발행될 때 사용.

[사전 확인]
  카메라 토픽 발행 여부:
    ros2 topic hz /camera/color/image_raw   (0Hz 이면 카메라 미동작 → stage4_full 사용)
  이미지 내용 확인:
    ros2 run rqt_image_view rqt_image_view /camera/color/image_raw

[동작 순서]
  0s  : apriltag_ros + mine_recorder + mine_pipeline 시작
  5s  : SLAM 시작
  15s : Nav2 시작
  50s : frontier_explorer_node 시작 (내부 start_delay 5s 포함)

[데이터 흐름]
  /camera/color/image_raw → apriltag_ros → /detections
    → mine_recorder_node → /mine_positions + CSV
      → mine_cluster_node → /mine_cluster_center
  /map → frontier_explorer_node → NavigateToPose
    → /finish_exploration → mine_goal_sender_node → 지뢰 군집 중심 이동
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

    # 실제 카메라 이미지 기반 AprilTag 인식
    # URDF의 libgazebo_ros_camera.so → /camera/color/image_raw → apriltag_ros → /detections
    # apriltag.launch.py 기본값이 이미 /camera/color/image_raw 로 설정되어 있음
    apriltag = _inc('sensor', 'apriltag.launch.py')

    # AprilTag 탐지 결과 → map 좌표 변환 + CSV 저장
    # 실제 카메라 사용이므로 camera_frame = 'camera_color_optical_frame'
    mine_recorder = Node(
        package='jackal_mine_detection',
        executable='mine_recorder_node',
        name='mine_recorder_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time':  True,
            'camera_frame':  'camera_color_optical_frame',  # 실제 카메라 frame
            'map_frame':     'map',
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
        apriltag,
        mine_recorder,
        _inc('mission', 'mine_pipeline.launch.py'),

        # ── SLAM [5s 후] ─────────────────────────────────────────────
        TimerAction(period=5.0,  actions=[_inc('sensor', 'slam.launch.py')]),

        # ── Nav2 [15s 후] ─────────────────────────────────────────────
        TimerAction(period=15.0, actions=[_inc('sensor', 'nav2.launch.py')]),

        # ── Frontier 탐사 [50s 후] ───────────────────────────────────
        TimerAction(period=50.0, actions=[frontier_explorer]),
    ])
