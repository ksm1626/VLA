# PROJECT.md

## 1. v1 Goal

v1 목표는 **공개 SO101/SO100 계열 데이터로 fine-tuned SmolVLA를 A6000에서 공식 LeRobot async 흐름으로 실행하고, SO101 로봇 PC의 ROS2 Gateway를 통해 실물 로봇을 안전하게 제어하는 파이프라인을 검증**하는 것이다.

성공 기준은 완벽한 pick-and-place 성공률이 아니라 다음 실행 경로 검증이다.

- A6000에서 SmolVLA fine-tuning과 offline inference 재현
- A6000에서 공식 LeRobot `PolicyServer` 실행
- A6000에서 공식 LeRobot `RobotClient`와 `RemoteSO101Robot` adapter 실행
- SO101 PC의 제조사 ROS2 bringup 무수정 유지
- SO101 단일 Python Gateway에서 로컬 ROS2 topic 구독/발행
- SSH tunnel + custom gRPC로 sensor/action packet 전달
- 모든 실물 action의 SO101-side safety gate 통과

공개 데이터만으로 실물 성공률은 낮을 수 있다. v1 acceptance criterion은 **안전한 fine-tuned policy 실행 파이프라인 검증**이다.

---

## 2. Architecture

```text
SO101 Robot PC
  제조사 ROS2 Humble bringup
  /front_cam, /top_cam, /joint_states
  /joint_targets, /safety/torque_enable
        |
        v
  so101_gateway/
    ROS2 Humble/rclpy
    local ROS2 subscribe/publish
    latest sensor buffer
    action queue / safety filter / watchdog
        |
        | custom gRPC over SSH tunnel
        v
A6000 Server
  remote_so101/
    RemoteSO101Robot adapter
    LeRobot Robot interface
        |
        v
  Official LeRobot RobotClient
        |
        | LeRobot native async gRPC
        v
  Official LeRobot PolicyServer
  fine-tuned SmolVLA checkpoint
  LeRobot SmolVLA fine-tuning
```

핵심 결정:

- ROS2는 SO101 로봇 PC 내부에서만 사용한다.
- A6000은 ROS2 topic을 직접 구독/발행하지 않는다.
- A6000에는 ROS2가 없어도 된다.
- SO101 PC는 제조사 bringup과 hardware control을 유지한다.
- SO101 연구용 코드는 우선 단일 Python gateway로 실행한다.
- A6000은 Python 3.12 LeRobot/SmolVLA 환경을 유지한다.
- 공식 LeRobot `RobotClient <-> PolicyServer` async 흐름은 유지한다.
- SO101 Gateway와 A6000 `RemoteSO101Robot` 사이만 custom gRPC로 구현한다.
- 실물 action 직전 safety는 SO101 PC에 둔다.

---

## 3. Environment Facts

SO101 로봇 PC:

- Jetson Orin Nano
- Jetson Linux 36.3.0
- ROS2 Humble source build
- Python 3.10.12
- 제조사 ROS2 패키지가 하드웨어 제어

A6000 서버:

- Ubuntu 22.04.5
- RTX A6000
- `vla-lerobot`: Python 3.12.13
- LeRobot requires `Python >=3.12`
- fine-tuning/checkpoint validation 완료
- ROS2 Humble apt 설치는 되어 있으나 v1 runtime에는 사용하지 않는다.

확인된 제약:

```text
ROS2 Humble rclpy: Python 3.10
LeRobot/SmolVLA: Python >=3.12
```

따라서 ROS2/rclpy와 LeRobot을 같은 Python process에 합치지 않는다.

---

## 4. Current Robot Interface

제조사 bringup:

```bash
source ~/physicai_arm_ws/install/setup.bash
ros2 launch physicai_arm bringup.launch.py
```

주요 ROS2 topic:

| Topic | Direction | Type | Note |
|---|---:|---|---|
| `/front_cam` | pub | `sensor_msgs/Image` | wrist/front 후보, 확인 필요 |
| `/top_cam` | pub | `sensor_msgs/Image` | top camera |
| `/joint_states` | pub | `sensor_msgs/JointState` | current state |
| `/joint_targets` | sub | `sensor_msgs/JointState` | command 의미 확인 필요 |
| `/safety/torque_enable` | sub | `std_msgs/Bool` | torque control |

SO101 Gateway는 같은 PC 안에서 위 topic을 구독/발행한다. ROS2 DDS를 A6000까지 직접 확장하지 않는다.

---

## 5. LeRobot/SmolVLA 기준

v1은 Hugging Face LeRobot 공식 문서와 native CLI/API를 1차 기준으로 따른다.

- base model: `lerobot/smolvla_base`
- current dataset: `lerobot/svla_so101_pickplace`
- server: official LeRobot `PolicyServer`
- client: official LeRobot `RobotClient`
- robot adapter: custom `RemoteSO101Robot`

현재 fine-tuned checkpoint:

```text
outputs/train/smolvla_so101/checkpoints/020000/pretrained_model
checkpoints/smolvla_so101 -> ../outputs/train/smolvla_so101/checkpoints/020000/pretrained_model
```

현재 checkpoint feature:

```text
observation.state: shape (6,)
observation.images.camera1
observation.images.camera2
observation.images.camera3
observation.images.empty_camera_0
action: shape (6,)
```

학습 때 사용한 rename 기준:

```yaml
policy.empty_cameras: 1
rename_map:
  observation.images.side: observation.images.camera1
  observation.images.up: observation.images.camera2
```

실물 camera mapping은 hard gate에서 확인 후 정한다. `/front_cam`은 실제 wrist인지 확인 전까지 wrist로 확정하지 않는다.

---

## 6. B안 Runtime Flow

Observation:

```text
SO101 ROS2 topics
  -> SO101 Gateway latest sensor buffer
  -> sensor packet
  -> SSH tunnel + custom gRPC
  -> A6000 RemoteSO101Robot.get_observation()
  -> LeRobot RobotClient
  -> LeRobot PolicyServer
  -> SmolVLA action chunk
```

Action:

```text
PolicyServer
  -> RobotClient
  -> RemoteSO101Robot.send_action()
  -> SSH tunnel + custom gRPC
  -> SO101 Gateway
  -> action queue
  -> safety filter
  -> /joint_targets
```

SO101에서 할 일:

- ROS2 topic 구독/발행
- 최신 sensor buffer 유지
- timestamp/freshness check
- image encoding 확인 및 RGB 변환
- joint name/order 정렬
- action queue, stale action guard
- NaN/Inf, joint limit, delta limit 검사
- watchdog state 관리

A6000에서 할 일:

- `RemoteSO101Robot`로 SO101 packet을 LeRobot observation으로 변환
- 공식 LeRobot `RobotClient` 실행
- 공식 LeRobot `PolicyServer` 실행
- LeRobot preprocessor/postprocessor 유지
- SmolVLA checkpoint inference

---

## 7. Hard Gates

실물 action 발행 전에 반드시 확인한다.

- `/joint_targets`가 absolute position target인지
- `/joint_states.name`과 `position`의 실제 joint 이름/순서
- joint 단위가 radian인지
- gripper 단위, open/close 방향, 안전 범위
- `/front_cam`의 실제 물리 위치
- `/top_cam`의 실제 물리 위치
- image encoding이 RGB/BGR/mono 중 무엇인지
- camera FPS, 해상도, compression 설정
- 제조사 controller의 control rate 및 내부 보간 여부
- 안전한 HOLD 방식
- SSH tunnel disconnect 시 action queue clear 및 HOLD/FAULT 동작
- SO101 단일 Python gateway에서 ROS2 topic 구독/발행 가능 여부

Hard gate 결과는 `docs/environment_audit.md`에 기록한다.

---

## 8. Implementation Status

완료:

- `vla-lerobot` conda 환경 생성
- `lerobot[smolvla]` 및 `grpcio` 설치
- SmolVLA fine-tuning wrapper 추가
- LeRobot PolicyServer wrapper 추가
- fine-tuning/policy server YAML config 추가
- ROS2 Humble 설치/검증 스크립트 추가
- 기존 `robot_client/` scaffold 추가
- fine-tuned checkpoint symlink 고정: `checkpoints/smolvla_so101`
- offline checkpoint validation script 추가
- fine-tuned SmolVLA checkpoint load 및 sample inference 검증 완료
- LeRobot PolicyServer startup 검증 완료
- `proto/so101_remote.proto` gRPC 계약 추가
- generated gRPC Python stub 추가
- A6000 `remote_so101/` bridge state/server 추가
- LeRobot `Robot` interface 기반 `RemoteSO101` adapter 추가
- A6000 mock SO101 Gateway 추가
- official LeRobot RobotClient + PolicyServer + mock Gateway 통합 검증 완료
- SO101 단일 Python Gateway scaffold 추가
- SO101 Gateway packet encoding/safety helper test 추가

현재 구현 파일:

```text
configs/finetune.smolvla.yaml
configs/policy_server.smolvla.yaml
configs/robot_client.so101.yaml
configs/remote_so101.yaml
configs/mock_gateway.so101.yaml
configs/so101_gateway.yaml
training/run_finetune.py
training/validate_lerobot_env.py
training/lerobot_config.py
policy/run_policy_server.py
policy/validate_checkpoint.py
robot_client/
remote_so101/
so101_gateway/
proto/
scripts/
tests/
```

주의:

- 기존 `robot_client/`는 이전 A6000-side ROS2 구조의 scaffold다.
- B안에서는 `robot_client/`를 그대로 확장하기보다 `so101_gateway/`, `remote_so101/`, `proto/` 중심으로 재정리한다.

검증 결과:

- dataset: `lerobot/svla_so101_pickplace`
- offline sample inference: raw action chunk `(1, 5, 6)`, postprocessed action chunk `(5, 6)`, finite values 확인
- PolicyServer startup: `0.0.0.0:8080` listen 확인
- official async mock integration: mock sensor packet -> `RemoteSO101Robot` -> official RobotClient -> official PolicyServer -> action chunk -> mock Gateway 수신 확인
- latest mock action packet shape: 6 joint targets, finite values 확인

---

## 9. Target Directory Direction

```text
configs/
training/
policy/
remote_so101/
so101_gateway/
proto/
scripts/
docs/
tests/
data/
outputs/
checkpoints/
logs/
```

역할:

- `policy/`: checkpoint validation, PolicyServer wrapper
- `remote_so101/`: A6000 `RemoteSO101Robot` adapter
- `so101_gateway/`: SO101 PC에서 실행할 단일 Python ROS2 Gateway
- `proto/`: custom gRPC schema
- `configs/`: training, policy server, gateway, remote robot 설정
- `tests/`: mock packet, gRPC round-trip, safety tests

---

## 10. Implementation Plan

### Phase 1. Fine-tuning

상태: 완료.

- `configs/finetune.smolvla.yaml` 기준으로 SmolVLA fine-tuning 실행
- checkpoint 생성 및 symlink 고정

### Phase 2. Offline Policy Validation

상태: 완료.

- `policy/validate_checkpoint.py`로 checkpoint load, dataset sample inference, action shape/finite 검증

### Phase 3. PolicyServer

상태: startup 검증 완료.

- 공식 LeRobot `policy_server` 실행 확인
- mock/sample client를 통한 handshake 및 action response 검증은 B안 adapter 구현 후 진행

### Phase 4. gRPC Contract

상태: 완료.

- `proto/so101_remote.proto` 설계
- sensor packet, action packet, heartbeat, status message 정의
- JPEG/raw RGB image metadata 포함
- sequence_id, timestamp_ns, timeout 정책 포함

완료 조건: mock client/server round-trip과 schema compatibility test 통과

### Phase 5. A6000 RemoteSO101Robot

상태: A6000 mock 기준 완료.

- LeRobot `Robot` 인터페이스 구현
- `get_observation()`에서 sensor packet을 LeRobot observation으로 변환
- `send_action()`에서 action packet을 SO101 Gateway로 전송
- 공식 LeRobot RobotClient가 사용할 config/registration 연결

완료 조건: mock SO101 Gateway와 공식 RobotClient <-> PolicyServer handshake 및 action chunk 수신 성공

### Phase 6. SO101 Gateway Read-only

- SO101 PC에서 ROS2 Humble/rclpy 단일 Python gateway 실행
- `/front_cam`, `/top_cam`, `/joint_states` 구독
- sensor packet 생성
- SSH tunnel을 통한 A6000 전송

상태: scaffold 및 ROS 없는 helper test 완료.

완료 조건: 실제 SO101 PC에서 10분 이상 sensor packet 수신 안정성 확인

### Phase 7. Dry-run Action Path

- A6000 action chunk가 SO101 Gateway까지 도착
- action queue, stale action guard, safety filter 검증
- 기본값 `actuation_enabled=false`
- `/joint_targets` publish 없음

완료 조건: invalid action reject와 disconnect HOLD/FAULT 확인

### Phase 8. Limited Real Robot Test

- 작은 단일 관절 이동
- 여러 관절 저속 이동
- 제한된 workspace에서 fine-tuned SmolVLA action 실행
- E-stop 및 수동 중단 절차 문서화

완료 조건: 모든 실물 action이 SO101-side safety gate를 통과해야 한다.

---

## 11. Future Work

- teleop 기반 자체 SO101 dataset 수집
- simulator client
- SO101 local safety relay 강화
- Zenoh/DDS Router/VPN+DDS 재검토
- ROS2 Humble-Jazzy 혼합 통신 재검토
- Pi0/Pi0.5 backend
- GR00T backend
- richer workspace guard
