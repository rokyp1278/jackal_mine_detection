#!/bin/bash
# ============================================================
# 00_check_env.sh
# Ubuntu 22.04 + ROS2 Humble + Gazebo + Jackal 환경 점검 스크립트
# 사용법: bash 00_check_env.sh
# ============================================================

set +e  # 일부 명령이 실패해도 계속 진행

echo "================================================================"
echo " Jackal Mine Detection - 환경 점검 시작"
echo "================================================================"

# 1. Ubuntu 버전
echo ""
echo "[1/8] Ubuntu 버전 확인"
echo "----------------------------------------------------------------"
lsb_release -a 2>/dev/null || echo "  lsb_release 없음 (sudo apt install lsb-release 필요)"

# 2. ROS2 환경 변수
echo ""
echo "[2/8] ROS2 환경 변수 확인"
echo "----------------------------------------------------------------"
echo "  ROS_DISTRO=${ROS_DISTRO:-(미설정)}"
echo "  ROS_VERSION=${ROS_VERSION:-(미설정)}"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0 (기본값)}"
if [ -z "$ROS_DISTRO" ]; then
  echo "  [경고] ROS2 환경이 source 되지 않았습니다."
  echo "         해결: source /opt/ros/humble/setup.bash"
fi

# 3. ROS2 실행파일 존재 여부
echo ""
echo "[3/8] ros2 실행파일 확인"
echo "----------------------------------------------------------------"
if command -v ros2 >/dev/null 2>&1; then
  echo "  [OK] ros2 명령 사용 가능: $(which ros2)"
  ros2 --help >/dev/null 2>&1 && echo "  [OK] ros2 정상 동작"
else
  echo "  [X] ros2 명령 없음. ROS2 Humble 설치가 필요합니다."
fi

# 4. Gazebo 설치
echo ""
echo "[4/8] Gazebo Classic 11 설치 확인"
echo "----------------------------------------------------------------"
if command -v gazebo >/dev/null 2>&1; then
  GZ_VER=$(gazebo --version 2>/dev/null | head -n1)
  echo "  [OK] gazebo 발견: $GZ_VER"
else
  echo "  [X] gazebo 명령 없음."
fi
if dpkg -l | grep -q "ros-humble-gazebo-ros-pkgs"; then
  echo "  [OK] ros-humble-gazebo-ros-pkgs 설치됨"
else
  echo "  [X] ros-humble-gazebo-ros-pkgs 미설치"
fi

# 5. Jackal 시뮬레이션 패키지
echo ""
echo "[5/8] Jackal 시뮬레이션 패키지 확인"
echo "----------------------------------------------------------------"
JACKAL_PKGS=(
  "ros-humble-jackal-description"
  "ros-humble-jackal-simulator"
  "ros-humble-jackal-gazebo"
  "ros-humble-jackal-navigation"
  "ros-humble-jackal-control"
  "ros-humble-jackal-msgs"
)
for pkg in "${JACKAL_PKGS[@]}"; do
  if dpkg -l | grep -q "^ii  $pkg "; then
    echo "  [OK] $pkg 설치됨"
  else
    echo "  [X] $pkg 미설치"
  fi
done

# 6. SLAM/Nav2 패키지
echo ""
echo "[6/8] SLAM Toolbox / Nav2 확인"
echo "----------------------------------------------------------------"
NAV_PKGS=(
  "ros-humble-slam-toolbox"
  "ros-humble-navigation2"
  "ros-humble-nav2-bringup"
  "ros-humble-nav2-msgs"
)
for pkg in "${NAV_PKGS[@]}"; do
  if dpkg -l | grep -q "^ii  $pkg "; then
    echo "  [OK] $pkg 설치됨"
  else
    echo "  [X] $pkg 미설치"
  fi
done

# 7. AprilTag 패키지 (2단계용)
echo ""
echo "[7/8] AprilTag 패키지 확인 (2단계용, 지금은 없어도 됩니다)"
echo "----------------------------------------------------------------"
APRIL_PKGS=(
  "ros-humble-apriltag"
  "ros-humble-apriltag-ros"
  "ros-humble-apriltag-msgs"
)
for pkg in "${APRIL_PKGS[@]}"; do
  if dpkg -l | grep -q "^ii  $pkg "; then
    echo "  [OK] $pkg 설치됨"
  else
    echo "  [경고] $pkg 미설치 (2단계에서 필요)"
  fi
done

# 8. colcon 빌드 도구
echo ""
echo "[8/8] colcon 빌드 도구 확인"
echo "----------------------------------------------------------------"
if command -v colcon >/dev/null 2>&1; then
  echo "  [OK] colcon 사용 가능: $(which colcon)"
else
  echo "  [X] colcon 미설치. 'sudo apt install python3-colcon-common-extensions' 필요"
fi

echo ""
echo "================================================================"
echo " 환경 점검 완료"
echo "================================================================"
echo ""
echo " 다음 단계:"
echo "   - [X] 항목이 있으면: bash 01_install_dependencies.sh 실행"
echo "   - 모두 [OK] 이면: bash 02_create_workspace.sh 실행"
echo ""
