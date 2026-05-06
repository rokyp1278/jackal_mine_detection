#!/bin/bash
# ============================================================
# 01_install_dependencies.sh
# Jackal Mine Detection 1단계 시뮬레이션을 위한 패키지 설치
# 사용법: bash 01_install_dependencies.sh
# ============================================================

set -e

echo "================================================================"
echo " 의존 패키지 설치 시작"
echo " (sudo 비밀번호를 1~2번 입력해야 할 수 있습니다)"
echo "================================================================"

# 1. apt 업데이트
echo ""
echo "[1/6] apt 업데이트"
echo "----------------------------------------------------------------"
sudo apt update

# 2. ROS2 Humble 기본 도구
echo ""
echo "[2/6] ROS2 Humble 기본 도구 확인"
echo "----------------------------------------------------------------"
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pip \
  build-essential \
  git

# rosdep 초기화 (이미 되어 있으면 스킵)
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init || true
fi
rosdep update

# 3. Gazebo Classic 11
echo ""
echo "[3/6] Gazebo Classic 11 + ros-gazebo bridge 설치"
echo "----------------------------------------------------------------"
sudo apt install -y \
  gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control

# 4. SLAM + Nav2
echo ""
echo "[4/6] SLAM Toolbox + Nav2 설치"
echo "----------------------------------------------------------------"
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-msgs \
  ros-humble-twist-mux

# 5. Jackal 시뮬레이션 패키지
echo ""
echo "[5/6] Jackal 시뮬레이션 패키지 설치"
echo "----------------------------------------------------------------"
sudo apt install -y \
  ros-humble-jackal-description \
  ros-humble-jackal-simulator \
  ros-humble-jackal-gazebo \
  ros-humble-jackal-navigation \
  ros-humble-jackal-control \
  ros-humble-jackal-msgs \
  ros-humble-clearpath-platform-msgs || {
    echo ""
    echo "  [경고] 일부 Jackal 패키지가 apt에 없을 수 있습니다."
    echo "         아래 fallback (소스 빌드) 안내를 참고하세요."
  }

# 6. AprilTag (2단계용, 미리 설치)
echo ""
echo "[6/6] AprilTag 패키지 설치 (2단계용)"
echo "----------------------------------------------------------------"
sudo apt install -y \
  ros-humble-apriltag \
  ros-humble-apriltag-ros \
  ros-humble-apriltag-msgs || echo "  [경고] AprilTag 일부 미설치 가능 - 2단계 진입 시 다시 시도"

# RealSense (2단계용, 옵션)
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-description || \
  echo "  [경고] RealSense 패키지 미설치 가능 - 2단계에서 필요시 별도 설치"

# Python 의존
pip3 install --user numpy

echo ""
echo "================================================================"
echo " 설치 완료. 다시 점검 스크립트를 돌려보세요:"
echo "    bash 00_check_env.sh"
echo "================================================================"
echo ""
echo " 만약 'ros-humble-jackal-*' 패키지가 apt에 없다면:"
echo " (Clearpath가 ROS2 Humble용 deb를 늦게 배포하는 경우가 있음)"
echo ""
echo "   1) Clearpath ROS2 저장소 추가가 필요할 수 있습니다."
echo "      https://docs.clearpathrobotics.com/docs/ros/installation/robot"
echo "      를 참고하거나, 소스로 빌드:"
echo ""
echo "      mkdir -p ~/jackal_ws/src && cd ~/jackal_ws/src"
echo "      git clone -b humble-devel https://github.com/jackal/jackal.git"
echo "      git clone -b humble-devel https://github.com/jackal/jackal_simulator.git"
echo "      cd ~/jackal_ws && rosdep install --from-paths src -i -y"
echo "      colcon build --symlink-install"
echo ""
