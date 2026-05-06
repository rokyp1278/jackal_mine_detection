#!/bin/bash
# ============================================================
# 03_setup_apriltag_textures.sh
# AprilTag 36h11 PNG 텍스처 이미지를 GitHub 에서 다운로드하여
# Gazebo 모델 디렉토리에 배치한다.
#
# 사용법:
#   cd ~/ros2_ws/src/jackal_mine_detection
#   bash scripts/03_setup_apriltag_textures.sh
#
# 결과:
#   models/apriltag_36h11_0/materials/textures/tag36_11_00000.png
#   models/apriltag_36h11_1/materials/textures/tag36_11_00001.png
#   models/apriltag_36h11_2/materials/textures/tag36_11_00002.png
#   models/apriltag_36h11_3/materials/textures/tag36_11_00003.png
# ============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
MODELS_DIR="$PKG_DIR/models"

BASE_URL="https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/tag36h11"

echo "========================================================"
echo " AprilTag 36h11 텍스처 다운로드"
echo " 대상 디렉토리: $MODELS_DIR"
echo "========================================================"

for TAG_ID in 0 1 2 3; do
    TAG_NAME=$(printf "tag36_11_%05d" "$TAG_ID")
    MODEL_DIR="$MODELS_DIR/apriltag_36h11_${TAG_ID}/materials/textures"
    PNG_FILE="$MODEL_DIR/${TAG_NAME}.png"

    mkdir -p "$MODEL_DIR"

    if [ -f "$PNG_FILE" ]; then
        echo "[태그 $TAG_ID] 이미 존재: $PNG_FILE"
        continue
    fi

    echo "[태그 $TAG_ID] 다운로드 중: ${TAG_NAME}.png"
    if curl -fsSL "${BASE_URL}/${TAG_NAME}.png" -o "$PNG_FILE"; then
        echo "[태그 $TAG_ID] ✓ 저장: $PNG_FILE"
    else
        echo "[태그 $TAG_ID] ✗ 다운로드 실패. 수동으로 PNG를 배치하세요."
        echo "   URL: ${BASE_URL}/${TAG_NAME}.png"
        echo "   대상: $PNG_FILE"
    fi
done

echo ""
echo "========================================================"
echo " 완료. Gazebo 실행 전 반드시 아래 확인:"
echo "   models/apriltag_36h11_0/materials/textures/tag36_11_00000.png"
echo "   models/apriltag_36h11_1/materials/textures/tag36_11_00001.png"
echo "   models/apriltag_36h11_2/materials/textures/tag36_11_00002.png"
echo "   models/apriltag_36h11_3/materials/textures/tag36_11_00003.png"
echo ""
echo " GAZEBO_MODEL_PATH 에 models/ 경로가 포함되어야 합니다."
echo " launch 파일이 자동으로 추가하지만, 수동 실행 시:"
echo "   export GAZEBO_MODEL_PATH=\$PWD/models:\$GAZEBO_MODEL_PATH"
echo "========================================================"
