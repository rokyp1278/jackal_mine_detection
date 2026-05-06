#!/bin/bash
# ============================================================
# 02_create_workspace.sh
# ~/jackal_ws ROS2 워크스페이스 생성 + jackal_mine_detection 패키지 위치 안내
# 사용법: bash 02_create_workspace.sh
# ============================================================

set -e

WS_DIR="$HOME/jackal_ws"

echo "================================================================"
echo " 워크스페이스 생성: $WS_DIR"
echo "================================================================"

mkdir -p "$WS_DIR/src"
cd "$WS_DIR"

# 빈 colcon 빌드 한 번 (디렉토리 구조 만들기)
if [ ! -d "$WS_DIR/install" ]; then
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install || true
fi

echo ""
echo "================================================================"
echo " 다음으로 jackal_mine_detection 패키지를 src/ 에 복사하세요."
echo "================================================================"
echo ""
echo "   현재 jackal_mine_detection 폴더 전체를 다음 위치로 복사:"
echo "     $WS_DIR/src/jackal_mine_detection"
echo ""
echo "   예시 (이 스크립트가 jackal_mine_detection/scripts/ 안에 있을 때):"
echo "     cp -r \"\$(dirname \$(realpath \$0))/..\"  $WS_DIR/src/jackal_mine_detection"
echo ""
echo "   복사 후 빌드:"
echo "     cd $WS_DIR"
echo "     source /opt/ros/humble/setup.bash"
echo "     colcon build --symlink-install --packages-select jackal_mine_detection"
echo "     source install/setup.bash"
echo ""
