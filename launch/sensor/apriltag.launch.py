"""
apriltag.launch.py
apriltag_ros 노드 실행.
카메라 토픽은 인자로 변경 가능.

기본값:
  image_topic:       /camera/color/image_raw
  camera_info_topic: /camera/color/camera_info
  tag_family:        36h11
  tag_size:          0.162 [m]  (실제 태그 물리 크기, mine world 의 모델과 일치해야 함)

사용:
  ros2 launch jackal_mine_detection apriltag.launch.py
  ros2 launch jackal_mine_detection apriltag.launch.py tag_size:=0.1
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',        default_value='true'),
        DeclareLaunchArgument('image_topic',         default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('camera_info_topic',   default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('tag_family',          default_value='36h11'),
        DeclareLaunchArgument('tag_size',            default_value='0.162'),

        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_ros',
            output='screen',
            remappings=[
                ('image_rect',   LaunchConfiguration('image_topic')),
                ('camera_info',  LaunchConfiguration('camera_info_topic')),
            ],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'family':       LaunchConfiguration('tag_family'),
                'size':         LaunchConfiguration('tag_size'),
                'max_hamming':  0,
                # detector 튜닝 (시뮬레이션 기본값)
                'detector.threads':  2,
                'detector.decimate': 1.0,
                'detector.blur':     0.0,
                'detector.refine':   True,
                'detector.sharpening': 0.25,
            }],
        ),
    ])
