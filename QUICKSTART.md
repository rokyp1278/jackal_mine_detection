# QUICKSTART (왕초보용 5분 시작)

이 문서대로만 따라하면 1단계 시뮬레이션이 동작합니다.

---

## 1) 패키지 폴더 받기

이 `jackal_mine_detection` 폴더 전체를 Ubuntu 22.04 머신의
`~/jackal_ws/src/` 안에 둡니다.

방법 예시 (USB / scp / 공유폴더 등 무엇이든 OK):

```bash
mkdir -p ~/jackal_ws/src
# 아래 경로는 본인 폴더로 바꿔주세요
cp -r /mnt/usb/jackal_mine_detection ~/jackal_ws/src/
ls ~/jackal_ws/src/jackal_mine_detection
# package.xml setup.py launch/ ... 가 보여야 정상
```

## 2) 환경 점검

```bash
cd ~/jackal_ws/src/jackal_mine_detection/scripts
chmod +x *.sh
bash 00_check_env.sh
```

`[X]` 가 있으면:

```bash
bash 01_install_dependencies.sh
bash 00_check_env.sh   # 다시 확인
```

## 3) 빌드

```bash
cd ~/jackal_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select jackal_mine_detection
```

`Summary: 1 package finished` 로그가 보이면 OK.

## 4) 가장 빠른 첫 테스트 (Gazebo 없이)

```bash
# === 터미널 A ===
source /opt/ros/humble/setup.bash
source ~/jackal_ws/install/setup.bash
ros2 launch jackal_mine_detection stage1_minimal.launch.py
```

```bash
# === 터미널 B (RViz) ===
source /opt/ros/humble/setup.bash
source ~/jackal_ws/install/setup.bash
rviz2 -d $(ros2 pkg prefix jackal_mine_detection)/share/jackal_mine_detection/config/rviz/mine_detection.rviz
```

확인 포인트:
- RViz 에 빨간 구 4개가 보임
- 군집 3개를 감싸는 파란 원이 보임
- 터미널 A 에 30초 뒤 `Publishing /trigger_final_goal` 출력

## 5) 빠르게 트리거하기 (30초 기다리기 싫을 때)

```bash
# === 터미널 C ===
source /opt/ros/humble/setup.bash
ros2 topic pub --once /finish_exploration std_msgs/Bool '{data: true}'
```

## 6) 토픽 직접 보기

```bash
ros2 topic echo /mine_positions
ros2 topic echo /mine_cluster_center
ros2 topic echo /exploration_state
```

## 7) Jackal + Gazebo 풀 시뮬레이션

`00_check_env.sh` 에서 Jackal 패키지가 모두 [OK] 일 때:

```bash
ros2 launch jackal_mine_detection stage1_fake_mine.launch.py
```

Gazebo 창이 뜨고, 30초 뒤 Jackal 이 fake 군집 중심으로 자동 이동.

---

문제가 생기면 `README.md` 의 "자주 만나는 문제" 섹션 확인.
