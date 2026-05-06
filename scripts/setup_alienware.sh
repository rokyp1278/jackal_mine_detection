#!/bin/bash
# ================================================================
# setup_alienware.sh
# Ubuntu 22.04 완전 초기 상태 → 개발 환경 전체 설치
#
# 사용법:
#   git clone <저장소URL> ~/jackal_mine_detection
#   cd ~/jackal_mine_detection/scripts
#   bash setup_alienware.sh
#
# 한 번만 실행하면 됩니다. 약 10~20분 소요.
# ================================================================
set -e

REPO_URL="https://github.com/rokyp1278/jackal_mine_detection.git"
WS="$HOME/ros2_ws"

echo "================================================================"
echo " Jackal Mine Detection 개발 환경 설치"
echo " Ubuntu 22.04 / ROS2 Humble"
echo "================================================================"
echo ""

# ── 0. 기본 도구 ─────────────────────────────────────────────────
echo "[0/7] 기본 도구 설치..."
sudo apt update -q
sudo apt install -y curl gnupg lsb-release software-properties-common git tmux openssh-server

# SSH 서버 활성화 (팀원 원격 접속용)
sudo systemctl enable ssh --now

# ── 1. ROS2 Humble 설치 ──────────────────────────────────────────
echo ""
echo "[1/7] ROS2 Humble 설치 (이미 설치된 경우 자동 스킵)..."
if [ ! -f /opt/ros/humble/setup.bash ]; then
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

    sudo add-apt-repository universe -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update -q
    sudo apt install -y ros-humble-desktop
    echo "  ROS2 Humble 설치 완료"
else
    echo "  ROS2 Humble 이미 설치됨 - 스킵"
fi

# ── 2. 시뮬레이션 패키지 ─────────────────────────────────────────
echo ""
echo "[2/7] 시뮬레이션 패키지 설치..."
sudo apt install -y \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-apriltag-ros \
    ros-humble-apriltag-msgs \
    ros-humble-realsense2-camera \
    python3-colcon-common-extensions \
    python3-rosdep

# ── 3. rosdep 초기화 ─────────────────────────────────────────────
echo ""
echo "[3/7] rosdep 초기화..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# ── 4. 워크스페이스 + 패키지 클론 ────────────────────────────────
echo ""
echo "[4/7] ROS2 워크스페이스 + 패키지 설정..."
mkdir -p "$WS/src"
if [ ! -d "$WS/src/jackal_mine_detection" ]; then
    git clone "$REPO_URL" "$WS/src/jackal_mine_detection"
    echo "  저장소 클론 완료"
else
    cd "$WS/src/jackal_mine_detection" && git pull
    echo "  저장소 최신화 완료"
fi

# ── 5. 빌드 ──────────────────────────────────────────────────────
echo ""
echo "[5/7] 패키지 빌드..."
source /opt/ros/humble/setup.bash
cd "$WS"
colcon build --packages-select jackal_mine_detection --symlink-install
echo "  빌드 완료"

# ── 6. bashrc 설정 ───────────────────────────────────────────────
echo ""
echo "[6/7] bashrc 환경 설정..."
BASHRC="$HOME/.bashrc"

add_if_missing() {
    grep -qF "$1" "$BASHRC" || echo "$1" >> "$BASHRC"
}

add_if_missing "source /opt/ros/humble/setup.bash"
add_if_missing "source $WS/install/setup.bash"
add_if_missing "export ROS_DOMAIN_ID=0"
add_if_missing "export GAZEBO_MODEL_PATH=$WS/install/jackal_mine_detection/share/jackal_mine_detection/models:\$GAZEBO_MODEL_PATH"

echo "  bashrc 설정 완료"

# ── 7. tmux 설정 ─────────────────────────────────────────────────
echo ""
echo "[7/7] tmux 설정..."
cat > "$HOME/.tmux.conf" << 'EOF'
# 마우스 스크롤 활성화
set -g mouse on

# 상태바 색상
set -g status-bg colour235
set -g status-fg colour136
set -g status-left '[#S] '
set -g status-right '#{=21:pane_title} %H:%M'

# 창 분할 단축키 (직관적으로)
bind | split-window -h -c "#{pane_current_path}"
bind - split-window -v -c "#{pane_current_path}"

# 창 이동 vim 스타일
bind h select-pane -L
bind j select-pane -D
bind k select-pane -U
bind l select-pane -R

# 히스토리 크기
set -g history-limit 10000

# 팀원 접속 시 창 크기 자동 조절
setw -g aggressive-resize on
EOF
echo "  tmux 설정 완료"

# ── 완료 ─────────────────────────────────────────────────────────
echo ""
echo "================================================================"
echo " 설치 완료!"
echo ""

# IP 주소 출력 (팀원 SSH 접속용)
IP=$(hostname -I | awk '{print $1}')
echo " 이 노트북 IP 주소: $IP"
echo ""
echo " 팀원 접속 방법:"
echo "   ssh $(whoami)@$IP"
echo "   tmux attach -t jackal    # 세션 참가"
echo ""
echo " 시뮬레이션 시작:"
echo "   bash ~/ros2_ws/src/jackal_mine_detection/scripts/start_collab.sh"
echo ""
echo " 새 터미널을 열거나 다음을 실행하세요:"
echo "   source ~/.bashrc"
echo "================================================================"
