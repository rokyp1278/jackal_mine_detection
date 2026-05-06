#!/bin/bash
# ================================================================
# start_collab.sh
# 팀 협업용 tmux 세션 시작 스크립트
#
# 사용법:
#   bash ~/ros2_ws/src/jackal_mine_detection/scripts/start_collab.sh
#
# 팀원 참가 방법 (SSH 접속 후):
#   tmux attach -t jackal
# ================================================================

SESSION="jackal"
WS="$HOME/ros2_ws"
SOURCE="source /opt/ros/humble/setup.bash && source $WS/install/setup.bash"

# 이미 세션이 있으면 그냥 참가
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "기존 세션 '$SESSION' 에 참가합니다..."
    tmux attach -t "$SESSION"
    exit 0
fi

echo "tmux 세션 '$SESSION' 시작..."

# ── 창 1: SIM (시뮬레이션 실행) ──────────────────────────────────
# 레이아웃:
#  ┌─────────────────┬─────────────────┐
#  │  Pane 0         │  Pane 1         │
#  │  Terminal 1     │  Terminal 2     │
#  │  (stage2_sim)   │  (stage4_full)  │
#  ├─────────────────┴─────────────────┤
#  │  Pane 2                           │
#  │  모니터링 (/mine_positions 등)      │
#  └───────────────────────────────────┘

tmux new-session -d -s "$SESSION" -n "SIM" -x 220 -y 50

# Pane 0: Terminal 1 (Gazebo)
tmux send-keys -t "$SESSION:SIM.0" "$SOURCE" Enter
tmux send-keys -t "$SESSION:SIM.0" "echo '=== Terminal 1: Gazebo 시작 ==='" Enter
tmux send-keys -t "$SESSION:SIM.0" "echo 'ros2 launch jackal_mine_detection stage2_sim.launch.py'" Enter

# Pane 1: Terminal 2 (파이프라인)
tmux split-window -h -t "$SESSION:SIM"
tmux send-keys -t "$SESSION:SIM.1" "$SOURCE" Enter
tmux send-keys -t "$SESSION:SIM.1" "echo '=== Terminal 2: 전체 파이프라인 ==='" Enter
tmux send-keys -t "$SESSION:SIM.1" "echo 'ros2 launch jackal_mine_detection stage4_full.launch.py'" Enter

# Pane 2: 모니터링
tmux split-window -v -t "$SESSION:SIM.0"
tmux send-keys -t "$SESSION:SIM.2" "$SOURCE" Enter
tmux send-keys -t "$SESSION:SIM.2" "echo '=== 모니터링: 토픽 확인 ==='" Enter
tmux send-keys -t "$SESSION:SIM.2" "echo 'ros2 topic echo /mine_positions'" Enter

# ── 창 2: DEV (개발/빌드) ────────────────────────────────────────
# 레이아웃:
#  ┌─────────────────┬─────────────────┐
#  │  Pane 0         │  Pane 1         │
#  │  코드 편집/git   │  빌드 터미널    │
#  └─────────────────┴─────────────────┘

tmux new-window -t "$SESSION" -n "DEV"
tmux send-keys -t "$SESSION:DEV" "cd $WS/src/jackal_mine_detection" Enter
tmux send-keys -t "$SESSION:DEV" "$SOURCE" Enter
tmux send-keys -t "$SESSION:DEV" "echo '=== DEV: 코드 편집 / git ==='" Enter

tmux split-window -h -t "$SESSION:DEV"
tmux send-keys -t "$SESSION:DEV.1" "cd $WS" Enter
tmux send-keys -t "$SESSION:DEV.1" "$SOURCE" Enter
tmux send-keys -t "$SESSION:DEV.1" "echo '=== 빌드: colcon build --packages-select jackal_mine_detection --symlink-install ==='" Enter

# SIM 창으로 포커스
tmux select-window -t "$SESSION:SIM"
tmux select-pane -t "$SESSION:SIM.0"

# ── 세션 참가 ─────────────────────────────────────────────────────
echo ""
echo "================================================================"
echo " tmux 세션 'jackal' 시작됨"
echo ""

IP=$(hostname -I | awk '{print $1}')
echo " 팀원 참가 방법:"
echo "   1. SSH 접속:  ssh $(whoami)@$IP"
echo "   2. 세션 참가: tmux attach -t jackal"
echo ""
echo " 창 이동: Ctrl+b → 숫자 (1=SIM, 2=DEV)"
echo " 창 분할 이동: Ctrl+b → 방향키"
echo " 스크롤: Ctrl+b → [ → 방향키 (q로 나가기)"
echo "================================================================"
echo ""

tmux attach -t "$SESSION"
