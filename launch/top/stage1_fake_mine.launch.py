"""
stage1_fake_mine.launch.py
======================================================================
1단계 풀 테스트.
Jackal Gazebo + SLAM + Nav2 + fake mine pipeline + goal sender 모두 실행.

전제:
  - ros-humble-jackal-* 패키지가 설치되어 있어야 함
  - ros-humble-slam-toolbox, ros-humble-nav2-bringup 설치되어 있어야 함
  - 만약 Jackal 패키지가 없다면 stage1_minimal.launch.py 부터 사용

사용법:
  source ~/jackal_ws/install/setup.bash
  ros2 launch jackal_mine_detection stage1_fake_mine.launch.py

기대 동작:
  1) Gazebo 가 빈 world + Jackal 로 켜진다
  2) slam_toolbox 가 /map 토픽을 publish 시작
  3) Nav2 stack 이 활성화 (/navigate_to_pose action 사용 가능)
  4) fake_mine_publisher_node 가 4개의 가짜 지뢰 좌표 publish
  5) mine_cluster_node 가 군집 중심 publish
  6) exploration_manager_node 가 30초 후 자동 /trigger_final_goal
     또는 사용자가 다음 명령으로 수동 종료:
         ros2 topic pub --once /finish_exploration std_msgs/Bool '{data: true}'
  7) mine_goal_sender_node 가 Nav2 goal 전송, Jackal 이 군집 근처로 이동

  ※ Jackal 의 Gazebo 모델이 spawn 된 직후 SLAM 이 map 을 만들기 시작하지만,
    아직 odom 만 있을 수 있다. mine_goal_sender 는 Nav2 가 활성화되기를
    기다리므로, Nav2 lifecycle activation 후 자동 진행된다.
======================================================================
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _try_pkg(pkg_name):
    try:
        return get_package_share_directory(pkg_name)
    except Exception:
        return None


def generate_launch_description():
    pkg_share = get_package_share_directory('jackal_mine_detection')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    actions = []

    # --- 1) Jackal Gazebo (있을 때만) ---
    jackal_gazebo_share = _try_pkg('jackal_gazebo')
    if jackal_gazebo_share is not None:
        # 최신 jackal_gazebo 의 launch 파일은 보통 jackal_world.launch.py
        launch_candidates = [
            ('jackal_gazebo', 'launch', 'jackal_world.launch.py'),
            ('jackal_gazebo', 'launch', 'gazebo.launch.py'),
            ('jackal_gazebo', 'launch', 'empty_world.launch.py'),
        ]
        chosen = None
        for pkg, sub, fname in launch_candidates:
            p = os.path.join(_try_pkg(pkg) or '', sub, fname)
            if os.path.exists(p):
                chosen = p
                break
        if chosen:
            actions.append(
                IncludeLaunchDescription(PythonLaunchDescriptionSource(chosen))
            )
        else:
            print('[stage1] WARNING: jackal_gazebo found but no known launch file')

    # --- 2) SLAM Toolbox (online_async) ---
    slam_share = _try_pkg('slam_toolbox')
    if slam_share is not None:
        slam_launch = os.path.join(
            slam_share, 'launch', 'online_async_launch.py'
        )
        if os.path.exists(slam_launch):
            actions.append(
                TimerAction(
                    period=5.0,
                    actions=[
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(slam_launch)
                        )
                    ],
                )
            )

    # --- 3) Nav2 (jackal_navigation 이 있으면 우선 사용) ---
    jackal_nav_share = _try_pkg('jackal_navigation')
    if jackal_nav_share is not None:
        nav_candidates = [
            os.path.join(jackal_nav_share, 'launch', 'navigation.launch.py'),
            os.path.join(jackal_nav_share, 'launch', 'nav2.launch.py'),
        ]
        chosen = next((p for p in nav_candidates if os.path.exists(p)), None)
        if chosen:
            actions.append(
                TimerAction(
                    period=8.0,
                    actions=[
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(chosen)
                        )
                    ],
                )
            )
    else:
        # nav2_bringup 의 navigation_launch.py 만 띄워보기 (map 없이는 한계 있음)
        nb = _try_pkg('nav2_bringup')
        if nb is not None:
            navlaunch = os.path.join(nb, 'launch', 'navigation_launch.py')
            if os.path.exists(navlaunch):
                actions.append(
                    TimerAction(
                        period=8.0,
                        actions=[
                            IncludeLaunchDescription(
                                PythonLaunchDescriptionSource(navlaunch)
                            )
                        ],
                    )
                )

    # --- 4) Mission pipeline ---
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'mission', 'fake_mine.launch.py')
            )
        )
    )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'mission', 'mine_pipeline.launch.py')
            )
        )
    )

    return LaunchDescription(actions)
