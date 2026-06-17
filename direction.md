# SO101-A6000 SmolVLA 프로젝트 결정사항

## 1. 현재 환경

### SO101 로봇 PC

- NVIDIA Jetson Orin Nano
- Jetson Linux 36.3.0
- ROS2 Humble source build
- Python 3.10.12
- 제조사 ROS2 패키지가 하드웨어 제어
- Wi-Fi 공유기에 연결
- 공유기는 외부 고정 IP를 할당받음
- 공유기 설정 권한 보유
- 기존 제조사 환경은 수정하지 않는 것이 원칙
- 연구용 코드는 우선 단일 Python gateway로 실행 예정

제조사 ROS2 주요 토픽:

```text
/front_cam
/top_cam
/joint_states
/joint_targets
/safety/torque_enable
```

### A6000 서버

- 서버실에 설치
- Ubuntu 22.04.5
- NVIDIA RTX A6000
- 고정 IP
- SO101 PC와는 다른 네트워크
- SSH 접속 가능
- 최신 LeRobot/SmolVLA 환경은 Python 3.12 사용
- 모델 학습, LeRobot RobotClient, LeRobot PolicyServer, SmolVLA 실시간 추론 담당

확인된 제약:

```text
ROS2 Humble rclpy: Python 3.10
LeRobot/SmolVLA: Python >=3.12
```

따라서 SO101 ROS2 환경과 A6000 LeRobot 환경을 한 Python 프로세스에 억지로 합치지 않는다.

---

## 2. 최종 선택한 아키텍처

v1은 **B안**으로 진행한다.

```text
SO101 제조사 ROS2
        |
        v
SO101 Python Gateway
  - ROS2 Humble/rclpy
  - /front_cam, /top_cam, /joint_states 구독
  - raw-ish sensor packet 생성
  - action 수신
  - action queue / safety filter / watchdog
  - /joint_targets 발행
        |
        | custom gRPC over SSH tunnel
        v
A6000 RemoteSO101Robot adapter
  - LeRobot Robot 인터페이스 구현
  - get_observation(): SO101 packet을 LeRobot observation으로 변환
  - send_action(): action을 SO101 Gateway로 전달
        |
        v
Official LeRobot RobotClient
        |
        | LeRobot native async gRPC
        v
Official LeRobot PolicyServer
        |
        v
fine-tuned SmolVLA checkpoint
```

핵심 결정:

- ROS2는 SO101 로봇 PC 내부에서만 사용한다.
- ROS2 토픽을 인터넷 또는 학교 네트워크를 통해 A6000까지 직접 전달하지 않는다.
- Zenoh Bridge, DDS Router, VPN+DDS, ROS2 Humble-Jazzy 혼합 통신은 v1에서 사용하지 않는다.
- SO101 PC에 ROS Gateway를 둔다.
- SO101 Gateway는 로컬 ROS2 topic을 구독하고 sensor packet을 만든다.
- A6000에는 ROS2가 없어도 된다.
- A6000에는 `RemoteSO101Robot` adapter를 둔다.
- 공식 LeRobot `RobotClient <-> PolicyServer` async 흐름은 유지한다.
- 실제 로봇 명령 직전의 action queue, safety filter, watchdog은 SO101 PC에 둔다.
- gRPC 연결은 SSH Local Port Forwarding으로 전달한다.

---

## 3. 공식 LeRobot 흐름과 v1 흐름

공식 LeRobot 흐름:

```text
LeRobot RobotClient
  - Robot 객체 생성
  - robot.connect()
  - robot.get_observation()
  - LeRobot async로 PolicyServer에 observation 전송
  - action chunk 수신
  - action queue / aggregation
  - robot.send_action()
        |
        v
LeRobot PolicyServer
  - checkpoint load
  - preprocessor/postprocessor
  - policy.predict_action_chunk()
```

v1 흐름:

```text
SO101 Gateway
  - ROS2 topic IO
  - safety/action publish
        |
        v
A6000 RemoteSO101Robot
  - LeRobot Robot처럼 동작
        |
        v
Official LeRobot RobotClient
        |
        v
Official LeRobot PolicyServer
```

차이점:

- 공식 RobotClient가 직접 로봇 하드웨어를 열지 않는다.
- A6000의 `RemoteSO101Robot`가 LeRobot `Robot` 인터페이스를 구현한다.
- 실제 ROS2 구독/발행과 safety는 SO101 Gateway가 담당한다.
- LeRobot의 async handshake, action chunk, queue/aggregation, PolicyServer inference 흐름은 최대한 재사용한다.

`RemoteSO101Robot`가 맞춰야 할 LeRobot 인터페이스:

```text
observation_features
action_features
is_connected
connect()
is_calibrated
calibrate()
configure()
get_observation()
send_action()
disconnect()
```

---

## 4. Observation 데이터 흐름

### 4.1 센서 데이터 생성

제조사 ROS2 노드가 다음 정보를 발행한다.

```text
/front_cam
/top_cam
/joint_states
```

### 4.2 SO101 Gateway가 topic 구독

```text
/front_cam ----+
               |
/top_cam ------+--> SO101 Gateway
               |
/joint_states -+
```

ROS callback은 최신 데이터를 buffer에 저장한다. Callback 안에서는 A6000 추론을 기다리지 않는다.

### 4.3 Sensor Packet 생성

SO101 Gateway는 최신 buffer에서 raw-ish sensor packet을 만든다.

```python
SensorPacket = {
    "top_image": top_or_resized_image_bytes,
    "front_image": front_or_resized_image_bytes,
    "joint_names": [...],
    "joint_positions": [...],
    "instruction": "...",
    "timestamp_ns": ...,
    "sequence_id": ...,
    "image_width": ...,
    "image_height": ...,
    "image_encoding": "rgb8" 또는 "jpeg-rgb",
}
```

SO101 Gateway에서 확인할 항목:

- 카메라 frame이 최신인가
- 관절 상태가 최신인가
- 두 카메라 timestamp 차이가 허용 범위 이내인가
- 이미지와 관절 상태 timestamp 차이가 허용 범위 이내인가
- 이미지 encoding이 RGB/BGR/mono 중 무엇인가
- RGB 변환이 올바른가
- joint name과 position 순서가 정확한가
- instruction이 비어 있지 않은가

### 4.4 A6000 RemoteSO101Robot이 LeRobot observation 생성

A6000 `RemoteSO101Robot.get_observation()`은 SO101 packet을 받아 LeRobot observation으로 변환한다.

현재 fine-tuned checkpoint 기준 feature:

```text
observation.state: shape (6,)
observation.images.camera1
observation.images.camera2
observation.images.camera3
observation.images.empty_camera_0
action: shape (6,)
```

학습 때 사용한 rename 기준:

```text
observation.images.side -> observation.images.camera1
observation.images.up   -> observation.images.camera2
policy.empty_cameras: 1
```

실물 camera mapping은 hard gate에서 확정한다. `/front_cam`을 wrist로 가정하지 않는다.

### 4.5 LeRobot 공식 preprocessor 유지

SO101에서 가능한 일:

```text
RGB/BGR 확인
JPEG 압축 또는 적당한 resize
timestamp/freshness check
joint order 정렬
```

A6000에서 유지할 일:

```text
LeRobot feature key 구성
LeRobot 공식 preprocessor
SmolVLA inference
LeRobot postprocessor
```

모델 입력 resize, normalize, tokenize 기준은 A6000의 LeRobot processor를 최종 기준으로 둔다.

---

## 5. 네트워크 흐름

기본 연결:

```text
SO101 Gateway
-> SO101 127.0.0.1:49100
-> SSH Local Port Forwarding
-> A6000 127.0.0.1:49100
-> RemoteSO101Robot adapter
```

원칙:

- SO101 공유기 포트포워딩을 사용하지 않는다.
- A6000 gRPC 포트를 외부에 공개하지 않는다.
- SSH tunnel이 끊기면 SO101 Gateway는 action queue를 비우고 HOLD/FAULT로 전환한다.
- 지연은 observation 조립 위치보다 image payload 크기, Wi-Fi 품질, SSH tunnel RTT, SmolVLA inference time에 더 크게 좌우된다.

초기 권장 payload:

```text
JPEG RGB 또는 resized RGB bytes
joint position 6개
timestamp
sequence_id
instruction
image metadata
```

full raw 640x480 RGB 2장은 30Hz에서 수백 Mbps까지 커질 수 있으므로 v1 기본값으로 쓰지 않는다.

---

## 6. Action 데이터 흐름

```text
PolicyServer
-> LeRobot RobotClient
-> RemoteSO101Robot.send_action()
-> custom gRPC over SSH tunnel
-> SO101 Gateway
-> action queue
-> safety filter
-> /joint_targets
```

SO101 Gateway의 publish 조건:

- `actuation_enabled=true`
- action이 NaN/Inf가 아님
- action timestamp가 stale하지 않음
- joint name/order가 일치함
- joint limit 통과
- per-step delta limit 통과
- watchdog state가 `READY` 또는 `RUNNING`

기본값은 dry-run이며 `/joint_targets`를 발행하지 않는다.

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
- SSH tunnel disconnect 시 동작
- SO101 단일 Python gateway에서 ROS2 topic 구독/발행 가능 여부

---

## 8. 현재 사용하지 않는 구조

v1에서는 다음을 사용하지 않는다.

```text
SO101 ROS2 <-> A6000 ROS2 직접 DDS 통신
Zenoh Bridge
DDS Router
Fast DDS Discovery Server
ROS2 Humble <-> ROS2 Jazzy 혼합 통신
A6000 측 ROS2/rclpy Gateway
SO101 공유기 포트포워딩
A6000 gRPC 포트 외부 공개
공식 LeRobot RobotClient를 SO101에서 직접 실행
custom SmolVLA inference server로 LeRobot PolicyServer를 대체
```

SO101 Gateway는 ROS2와 안전을 담당한다. A6000 `RemoteSO101Robot`는 공식 LeRobot RobotClient가 사용할 가상 로봇 역할을 한다.

---

## 9. 구현 순서

1. `PROJECT.md`와 구현 디렉토리를 B안 기준으로 정리: 완료
2. custom gRPC proto 설계: sensor packet, action packet, heartbeat: 완료
3. SO101 Gateway skeleton 구현: 단일 Python gateway scaffold 완료
4. A6000 `RemoteSO101Robot` adapter 구현: 완료
5. 공식 LeRobot RobotClient가 `RemoteSO101Robot`를 사용할 수 있게 registration/config 연결: 완료
6. A6000 localhost mock packet으로 RobotClient <-> PolicyServer handshake 검증: 완료
7. SO101 host에서 read-only ROS2 topic 구독 검증
8. SSH tunnel에서 sensor packet round-trip 검증
9. dry-run action chunk 수신 검증
10. safety-gated `/joint_targets` publish 구현
11. 제한된 단일 관절 실물 테스트

---

## 10. 최종 결정 요약

```text
SO101 제조사 ROS2
        |
        v
SO101 Python Gateway
        |
        | custom gRPC over SSH tunnel
        v
A6000 RemoteSO101Robot adapter
        |
        v
Official LeRobot RobotClient
        |
        | LeRobot native async gRPC
        v
Official LeRobot PolicyServer
        |
        v
fine-tuned SmolVLA
        |
        v
LeRobot action chunk
        |
        v
RemoteSO101Robot.send_action()
        |
        | custom gRPC over SSH tunnel
        v
SO101 Gateway
        |
        v
Action Queue -> Safety Filter -> /joint_targets
```

핵심 원칙:

```text
ROS2는 SO101 로컬에서만 사용
네트워크에는 sensor packet과 action packet만 전달
A6000은 공식 LeRobot async 흐름을 최대한 유지
SO101가 실제 제어와 안전을 담당
SSH는 custom gRPC 포트를 연결하는 통로
```
