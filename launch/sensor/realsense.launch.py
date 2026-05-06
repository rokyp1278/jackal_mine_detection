"""
realsense.launch.py
Intel RealSense D435 카메라 드라이버 실행.

사전 조건:
  sudo apt install ros-humble-realsense2-camera

발행 토픽 (apriltag.launch.py 기본값과 일치):
  /camera/color/image_raw
  /camera/color/camera_info
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[{
                'use_sim_time':   False,
                'enable_color':   True,
                'enable_depth':   False,
                'color_width':    640,
                'color_height':   480,
                'color_fps':      30,
                'enable_gyro':    False,
                'enable_accel':   False,
            }],
        ),
    ])
