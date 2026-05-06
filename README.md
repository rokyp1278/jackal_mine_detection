# jackal_mine_detection

ROS2 Humble 기반 Jackal UGV 실내 지뢰(AprilTag) 자율 탐지 시스템.

## 현재 상태 (2026-05-06 기준)

| 단계 | 내용 | 상태 |
|---|---|---|
| 시뮬레이션 | Gazebo + SLAM + Nav2 + Frontier 자율탐사 + 지뢰탐지 + 목표이동 | ✅ 완료 |
| 하드웨어 | Jackal + Jetson NX + RealSense + AprilTag 실제 인식 | ⬜ 미완 |

---

## 시스템 구성

```
[Jetson NX]
  ├─ RealSense D435  →  /camera/color/image_raw
  ├─ 2D LiDAR        →  /scan  (토픽 이름 확인 필요 → 아래 TODO 참고)
  ├─ slam_toolbox    →  /map
  ├─ Nav2            →  /cmd_vel
  ├─ frontier_explorer_node
  ├─ mine_recorder_node  (AprilTag → map 좌표 저장)
  ├─ mine_cluster_node
  ├─ mine_goal_sender_node
  └─ exploration_manager_node

[Jackal mini PC]
  └─ jackal_robot driver  →  /odom 발행, /cmd_vel 수신 → 모터 제어
```

두 PC는 **같은 ROS_DOMAIN_ID**, **같은 네트워크 서브넷**에 있어야 합니다.

---

## 의존 패키지 설치

### Jetson NX

```bash
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-realsense2-camera
sudo apt install ros-humble-apriltag-ros
sudo apt install ros-humble-apriltag-msgs
```

### Jackal mini PC

```bash
# Clearpath 공식 jackal 드라이버 (모터 제어)
sudo apt install ros-humble-jackal-robot
# 소스 빌드 방법: https://github.com/clearpathrobotics/jackal_robot
```

---

## 빌드 (Jetson NX)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <이 저장소 URL> jackal_mine_detection
cd ~/ros2_ws
colcon build --packages-select jackal_mine_detection --symlink-install
source install/setup.bash
```

---

## 실행

### 시뮬레이션 (WSL 환경)

```bash
# 터미널 1
ros2 launch jackal_mine_detection stage2_sim.launch.py

# 터미널 2 — Gazebo에 노란 로봇이 보인 후 실행
ros2 launch jackal_mine_detection stage4_full.launch.py
```

### 실제 하드웨어

```bash
# Jackal mini PC
ros2 launch jackal_robot jackal_robot.launch.py

# Jetson NX — Jackal PC가 올라온 후 실행
ros2 launch jackal_mine_detection hw_frontier.launch.py
```

---

## 동작 순서 (자동)

```
[0s]  지뢰 탐지 노드 + 파이프라인 시작
[5s]  SLAM 시작
[15s] Nav2 시작
[50s] Frontier 자율 탐사 시작
      → 미지 공간 자동 탐사
      → AprilTag 3m 이내 접근 시 자동 탐지
      → 탐사 완료 → /finish_exploration 발행
      → 지뢰 군집 중심으로 자동 이동 → Goal succeeded
```

---

## ⚠️ 하드웨어 적용 전 반드시 확인할 것

### 1. LiDAR 토픽 이름 (가장 중요)

```bash
# Jackal 연결 후 실행
ros2 topic list | grep scan
```

결과에 따라 `config/slam_params.yaml` 수정:

```yaml
# 현재 (시뮬 기본값)
scan_topic: /scan

# Hokuyo인 경우
# scan_topic: /front/scan

# Velodyne인 경우 2D scan으로 변환 후
# scan_topic: /scan
```

### 2. AprilTag 실물 크기 확인

`launch/sensor/apriltag.launch.py`의 `tag_size` 기본값이 `0.162`(m)로 설정되어 있습니다.
실제 출력한 태그 크기와 다르면 pose 계산이 틀립니다.

```bash
# 태그 크기 지정하여 실행하는 방법
ros2 launch jackal_mine_detection hw_frontier.launch.py  # 내부에서 apriltag.launch.py 호출
# apriltag.launch.py 의 tag_size 파라미터를 실제 크기로 수정할 것
```

### 3. ROS_DOMAIN_ID 통일

Jetson NX와 Jackal mini PC 양쪽 `~/.bashrc`에 동일하게 추가:

```bash
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc
```

### 4. RealSense 카메라 TF 확인

```bash
# hw_frontier 실행 중에
ros2 run rqt_tf_tree rqt_tf_tree
# camera_color_optical_frame 이 TF 트리에 있는지 확인
```

---

## 주요 파라미터 (`config/params.yaml`)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `detect_range_m` | 3.0 | 지뢰 탐지 거리 (m) |
| `cluster_radius` | 1.0 | 군집 반경 (m) |
| `min_frontier_size` | 8 | 최소 frontier 크기 (노이즈 제거) |
| `goal_timeout_sec` | 60.0 | 단일 goal 타임아웃 (s) |
| `approach_offset` | 1.5 | 지뢰 군집 접근 거리 (m) |

---

## 주요 launch 파일

| 파일 | 용도 |
|---|---|
| `launch/top/stage2_sim.launch.py` | 시뮬레이션 환경 (Gazebo + 로봇) |
| `launch/top/stage4_full.launch.py` | 시뮬레이션 전체 파이프라인 |
| `launch/top/hw_frontier.launch.py` | **하드웨어 전용** 전체 파이프라인 |
| `launch/top/stage4_camera.launch.py` | 실제 카메라 인식 테스트용 |
