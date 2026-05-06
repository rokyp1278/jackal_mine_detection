"""
gazebo.launch.py
Gazebo Classic 시뮬레이션 시작 + jackal_sim 로봇 스폰.
jackal_description / jackal_gazebo 패키지 없이 자체 URDF 사용.

토픽 출력:
  /scan              sensor_msgs/LaserScan
  /camera/color/image_raw    sensor_msgs/Image
  /camera/color/camera_info  sensor_msgs/CameraInfo
  /odom              nav_msgs/Odometry
  /cmd_vel           geometry_msgs/Twist  (입력)

TF 출력:
  odom → base_link   (diff drive plugin)
  base_link → laser  (robot_state_publisher)
  base_link → camera_link → camera_color_optical_frame  (robot_state_publisher)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, SetEnvironmentVariable, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('jackal_mine_detection')

    # ── 환경변수 설정 ─────────────────────────────────────────────────
    models_path = os.path.join(pkg_share, 'models')
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', ''),
    )
    # gazebo_ros 플러그인 경로 명시 (WSL 환경변수 미전달 방지)
    set_plugin_path = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        '/opt/ros/humble/lib:' + os.environ.get('GAZEBO_PLUGIN_PATH', ''),
    )
    # WSL/Mesa 소프트웨어 렌더링 (GPU 드라이버 문제 방지)
    set_mesa   = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE',    '1')
    # Ogre off-screen 렌더링 모드 (WSL 에서 카메라 센서 활성화 필수)
    set_ogre   = SetEnvironmentVariable('OGRE_RTT_MODE',             'Copy')
    set_oglver = SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE',  '3.3')

    # ── Gazebo 시작 (mine_detection.world) ───────────────────────────
    world_file = os.path.join(pkg_share, 'worlds', 'mine_detection.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py',
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
        }.items(),
    )

    # ── URDF 읽기 → robot_description 파라미터 ───────────────────────
    urdf_file = os.path.join(pkg_share, 'urdf', 'jackal_sim.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ── robot_state_publisher: URDF → TF + /robot_description ────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
        }],
    )

    # ── spawn_entity: 5초 후 Gazebo 에 로봇 스폰 ─────────────────────
    # WSL 에서 Gazebo 서버가 완전히 초기화되기를 기다림
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_jackal',
                arguments=[
                    '-entity', 'jackal_sim',
                    '-topic',  '/robot_description',
                    '-x', '1.5',
                    '-y', '0.0',
                    '-z', '0.13',
                    '-Y', '0.0',
                ],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        set_mesa,
        set_ogre,
        set_oglver,
        set_plugin_path,
        set_model_path,
        gazebo,
        robot_state_pub,
        spawn_robot,
    ])
