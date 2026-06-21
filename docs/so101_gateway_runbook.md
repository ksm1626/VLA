# SO101 Gateway Runbook

이 문서는 SO101 Robot PC에서 A6000 SmolVLA policy pipeline에 연결하기 위한 실행 순서다.

기본 원칙:

- SO101에서는 LeRobot, torch, conda를 설치하지 않는다.
- SO101에서는 제조사 ROS2 Humble 환경과 단일 Python gateway만 사용한다.
- `apt upgrade`, `torch`, `numpy`, `opencv` 재설치는 하지 않는다.
- 처음에는 반드시 dry-run으로 실행한다.
- dry-run에서는 action을 받아도 `/follower/joint_targets`를 publish하지 않는다.

GitHub repo:

```bash
https://github.com/ksm1626/VLA/tree/main
```

---

## 1. Quick Ref 기준

SO101 Robot PC 정보:

- Jetson Orin Nano
- Jetson Linux 36.3.0
- Linux kernel 5.15.136-tegra
- ROS2 Humble, source build
- Python 3.10.12
- OpenCV 4.10.0, cuda/gstreamer
- numpy 1.26.1
- torch 2.4.0a0, 하지만 gateway에서는 torch를 쓰지 않음

제조사 ROS2 topic:

| Topic | Pub/Sub | Type | 용도 |
|---|---:|---|---|
| `/arm/front_cam` | Pub | `sensor_msgs/Image` | 손목/Eye-in-Hand 카메라 |
| `/arm/top_cam` | Pub | `sensor_msgs/Image` | 상단 카메라 |
| `/follower/joint_states` | Pub | `sensor_msgs/JointState` | 현재 관절 상태 |
| `/follower/joint_targets` | Sub | `sensor_msgs/JointState` | absolute 목표 관절각 |
| `/tf` | Pub | `tf2_msgs/TFMessage` | TF |
| `/follower/safety/torque_enable` | Sub | `std_msgs/Bool` | torque enable/disable |

Joint limits:

| Joint | Range |
|---|---:|
| `shoulder_pan` | `-1.92 ~ 1.92` |
| `shoulder_lift` | `-1.75 ~ 1.75` |
| `elbow_flex` | `-1.69 ~ 1.69` |
| `wrist_flex` | `-1.66 ~ 1.66` |
| `wrist_roll` | `-2.74385 ~ 2.84121` |
| `gripper` | `-0.174533 ~ 1.74533` |

---

## 2. 전체 흐름

```text
SO101 제조사 ROS2 bringup
  /arm/front_cam
  /arm/top_cam
  /follower/joint_states
        |
        v
so101_sensor_gateway.py
  - 카메라/관절값 구독
  - 이미지 RGB 변환 + JPEG 압축
  - SensorPacket 생성
  - SSH tunnel을 통해 A6000으로 gRPC 전송
  - ActionPacket 수신
  - dry-run이면 출력만 함
  - actuation_enabled=true이면 safety 통과 후 /follower/joint_targets 발행
        |
        v
A6000 RemoteSO101 bridge
        |
        v
Official LeRobot RobotClient
        |
        v
Official LeRobot PolicyServer
        |
        v
fine-tuned SmolVLA checkpoint
```

---

## 3. SO101에 repo 받기

```bash
cd ~
git clone https://github.com/ksm1626/VLA.git
cd VLA
```

이미 받았다면:

```bash
cd ~/VLA
git pull
```

---

## 4. 제조사 ROS2 환경 확인

```bash
source ~/physicai_arm_ws/install/setup.bash
ros2 topic list
python3 --version
```

정상 기준:

```text
Python 3.10.12
ROS2 Humble
```

만약 `ros2`가 안 잡히면 아래 순서로 다시 시도한다.

```bash
source ~/ros2_base/install/setup.bash
source ~/physicai_arm_ws/install/setup.bash
ros2 topic list
```

---

## 5. Gateway Python 환경 준비

제조사 Python/ROS 패키지를 그대로 쓰기 위해 `--system-site-packages` venv를 사용한다.

```bash
cd ~/VLA
python3 -m venv ~/vla_gateway_env --system-site-packages
source ~/vla_gateway_env/bin/activate
python -m pip install "grpcio==1.73.1" "protobuf==6.31.1" PyYAML
```

의존성 확인:

```bash
source ~/physicai_arm_ws/install/setup.bash
source ~/vla_gateway_env/bin/activate
cd ~/VLA

python - <<'PY'
import rclpy
import sensor_msgs
import cv2
import numpy
import grpc
import google.protobuf
from remote_so101.proto_modules import pb2, pb2_grpc

print("protobuf", google.protobuf.__version__)
print("SO101 gateway deps OK")
PY
```

주의:

- `numpy`, `opencv`, `torch`는 업그레이드하지 않는다.
- SO101 gateway는 torch를 사용하지 않는다.
- LeRobot은 SO101에 설치하지 않는다.

---

## 6. 제조사 bringup 실행

터미널 1:

```bash
source ~/physicai_arm_ws/install/setup.bash
ros2 launch physicai_arm bringup.launch.py
```

이 명령은 manipulator, camera, TF2를 켠다.

---

## 7. ROS topic 확인

터미널 2:

```bash
source ~/physicai_arm_ws/install/setup.bash

ros2 topic list
ros2 topic info /arm/front_cam
ros2 topic info /arm/top_cam
ros2 topic info /follower/joint_states
ros2 topic info /follower/joint_targets
ros2 topic echo /follower/joint_states --once
```

확인할 것:

```text
/arm/front_cam             sensor_msgs/Image
/arm/top_cam               sensor_msgs/Image
/follower/joint_states     sensor_msgs/JointState
/follower/joint_targets    sensor_msgs/JointState
```

`/follower/joint_states.name`이 아래 순서와 같은지 확인한다.

```text
shoulder_pan
shoulder_lift
elbow_flex
wrist_flex
wrist_roll
gripper
```

다르면 `configs/so101_gateway.yaml`의 `sensor.joint_names`를 실제 이름/순서에 맞춘다.

---

## 8. SO101 Gateway Config 수정

파일:

```text
configs/so101_gateway.yaml
```

기본값은 안전하게 action 발행이 꺼져 있다.

```yaml
actions:
  actuation_enabled: false
```

실물 발행 전에는 joint limit을 반드시 채운다.

```yaml
safety:
  limits_required_for_actuation: true
  max_delta_per_step: 0.05
  joint_limits:
    shoulder_pan: [-1.91986, 1.91986]
    shoulder_lift: [-1.74533, 1.74533]
    elbow_flex: [-1.69, 1.69]
    wrist_flex: [-1.65806, 1.65806]
    wrist_roll: [-2.74385, 2.84121]
    gripper: [-0.174533, 1.74533]
```

처음 실물 테스트에서는 `max_delta_per_step`을 더 작게 시작하는 것을 권장한다.

```yaml
max_delta_per_step: 0.01
```

---

## 9. A6000 SSH Tunnel 열기

SO101에서 A6000 bridge에 붙기 위해 SSH local port forwarding을 연다.

터미널 3:

```bash
ssh -N -L 49100:127.0.0.1:49100 <A6000_USER>@<A6000_IP>
```

의미:

```text
SO101 127.0.0.1:49100
  -> SSH tunnel
  -> A6000 127.0.0.1:49100
```

따라서 `configs/so101_gateway.yaml`의 bridge는 기본값 그대로 둔다.

```yaml
bridge:
  host: 127.0.0.1
  port: 49100
```

---

## 10. A6000 쪽 준비 상태

A6000에서는 다음이 떠 있어야 한다.

터미널 A:

```bash
conda activate vla-lerobot
cd ~/VLA
python policy/run_policy_server.py \
  --config configs/policy_server.smolvla.yaml
```

터미널 B:

```bash
conda activate vla-lerobot
cd ~/VLA
python remote_so101/run_robot_client.py \
  --config configs/remote_so101.yaml \
  --duration-s 60 \
  --max-steps 300 \
  --stop-after-actions 1
```

`run_robot_client.py`가 A6000의 `127.0.0.1:49100`에서 SO101 bridge gRPC server를 연다.

---

## 11. SO101 Gateway Dry-run 실행

터미널 4:

```bash
source ~/physicai_arm_ws/install/setup.bash
source ~/vla_gateway_env/bin/activate
cd ~/VLA

python so101_gateway/so101_sensor_gateway.py \
  --config configs/so101_gateway.yaml \
  --dry-run
```

이 단계에서 일어나는 일:

```text
/arm/front_cam, /arm/top_cam, /follower/joint_states 구독
이미지 RGB 변환
이미지 JPEG 압축
SensorPacket 생성
A6000으로 전송
A6000에서 ActionPacket 수신
DRY-RUN action ... 출력
/follower/joint_targets 발행 안 함
```

정상 로그 예:

```text
SO101 gateway started: bridge=127.0.0.1:49100 actuation_enabled=False
DRY-RUN action sequence_id=1 targets=[...]
```

---

## 12. SO101에 필요한 최소 파일 구조

SO101에서 실제 gateway 실행에 필요한 최소 구조:

```text
~/VLA/
├── configs/
│   └── so101_gateway.yaml
├── so101_gateway/
│   ├── __init__.py
│   └── so101_sensor_gateway.py
├── remote_so101/
│   ├── __init__.py
│   ├── config_loader.py
│   └── proto_modules.py
└── proto/
    └── generated/
        ├── __init__.py
        ├── so101_remote_pb2.py
        └── so101_remote_pb2_grpc.py
```

전체 repo를 clone하면 위 파일은 모두 포함된다.

SO101에서 필요 없는 디렉토리:

- `policy/`
- `training/`
- `outputs/`
- `checkpoints/`
- `tests/`

있어도 문제는 없지만 실제 SO101 runtime에는 사용하지 않는다.

---

## 13. 실물 발행 전 Hard Gate

다음이 확인되기 전에는 `actuation_enabled: true`로 바꾸지 않는다.

- `/follower/joint_targets`가 absolute position target인지
- `/follower/joint_states.name`과 position 배열 순서
- joint 단위가 radian인지
- gripper open/close 방향
- gripper 단위와 안전 범위
- `/arm/front_cam` 실제 물리 위치
- `/arm/top_cam` 실제 물리 위치
- image encoding이 `rgb8`, `bgr8`, `mono8`, `rgba8`, `bgra8` 중 무엇인지
- camera FPS와 해상도
- 제조사 controller의 control rate
- 내부 보간 여부
- SSH tunnel 끊김 시 action queue 처리
- 수동 E-stop 또는 torque disable 절차

---

## 14. 실물 발행으로 넘어갈 때

먼저 config를 바꾼다.

```yaml
actions:
  actuation_enabled: true
```

그리고 safety limit이 모두 채워져 있어야 한다.

처음 실물 테스트는 policy action이 아니라 작은 단일 관절 테스트로 시작한다.

추천 순서:

1. torque enable/disable 수동 절차 확인
2. `/follower/joint_targets`에 현재 위치와 거의 같은 값 publish
3. 단일 joint를 `0.005 ~ 0.01 rad` 수준으로만 이동
4. 여러 joint 저속 이동
5. dry-run action 값을 사람이 확인
6. 제한된 workspace에서 SmolVLA action 실행

---

## 15. 문제 발생 시 확인

`rclpy` import 실패:

```bash
source ~/physicai_arm_ws/install/setup.bash
python3 -c "import rclpy; print('ok')"
```

protobuf 버전 mismatch:

```bash
source ~/vla_gateway_env/bin/activate
python -m pip install "protobuf==6.31.1" "grpcio==1.73.1"
```

camera frame이 안 들어오는 경우:

```bash
ros2 topic hz /arm/front_cam
ros2 topic hz /arm/top_cam
ros2 topic echo /arm/front_cam --once
```

joint state가 안 들어오는 경우:

```bash
ros2 topic hz /follower/joint_states
ros2 topic echo /follower/joint_states --once
```

A6000 연결 실패:

```bash
ssh -N -L 49100:127.0.0.1:49100 <A6000_USER>@<A6000_IP>
```

A6000 쪽에서 `remote_so101/run_robot_client.py`가 실행 중인지 확인한다.
