"""
stage1_minimal.launch.py
======================================================================
가장 단순한 1단계 테스트. Gazebo / SLAM / Nav2 없이도 동작.
fake_mine_publisher_node + mine_cluster_node + exploration_manager_node
+ map static TF 만 띄운다.

목적:
  - 우리가 만든 4개 노드 중 3개 (Nav2 빼고) 가 정상적으로 데이터를
    주고받는지 시각화로 빠르게 확인.
  - Nav2 가 없으니 mine_goal_sender_node 는 띄우지 않는다.
  - RViz 만 따로 띄워서 /mine_markers, /mine_cluster_marker 확인.

사용법:
  ros2 launch jackal_mine_detection stage1_minimal.launch.py
  (다른 터미널)
  rviz2 -d $(ros2 pkg prefix jackal_mine_detection)/share/jackal_mine_detection/config/rviz/mine_detection.rviz
======================================================================
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('jackal_mine_detection')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # stage1 은 Gazebo 없음 → /clock 없음 → use_sim_time=False 강제
    # (params.yaml 의 use_sim_time: true 를 여기서 덮어씀)
    sim_override = {'use_sim_time': False}

    fake_mine_node = Node(
        package='jackal_mine_detection',
        executable='fake_mine_publisher_node',
        name='fake_mine_publisher_node',
        output='screen',
        parameters=[params_file, sim_override],
    )
    cluster_node = Node(
        package='jackal_mine_detection',
        executable='mine_cluster_node',
        name='mine_cluster_node',
        output='screen',
        parameters=[params_file, sim_override],
    )
    manager_node = Node(
        package='jackal_mine_detection',
        executable='exploration_manager_node',
        name='exploration_manager_node',
        output='screen',
        parameters=[params_file, sim_override],
    )

    # Nav2 / SLAM 없이도 RViz Fixed Frame=map 이 동작하도록
    # world -> map static TF 를 임시로 publish.
    map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen',
    )

    return LaunchDescription([
        map_tf,
        fake_mine_node,
        cluster_node,
        manager_node,
    ])
