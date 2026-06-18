# SO101–A6000 VLA End-to-End Dry-run 진행 결과

## 1. 프로젝트 구조

현재 확정 및 검증 중인 구조는 다음과 같다.

```text
SO101 Robot PC
- 제조사 ROS2 bringup
- 카메라 및 관절 상태 수집
- SensorPacket 생성
- SSH tunnel을 통해 A6000에 전송
- A6000 ActionPacket 수신
- Dry-run에서는 Action 출력만 수행
- 실제 제어 시 Safety Filter 후 joint target 발행

        ↕ SSH Local Port Forwarding / gRPC

A6000 Server
- RemoteSO101 bridge
- 공식 LeRobot RobotClient
- 공식 LeRobot PolicyServer
- Fine-tuned SmolVLA checkpoint
```

사용 포트:

```text
49100:
SO101 Gateway ↔ A6000 RemoteSO101 bridge

8080:
A6000 RobotClient ↔ LeRobot PolicyServer
```

---

## 2. SO101 환경

SO101 PC 환경:

```text
NVIDIA Jetson Orin Nano
Jetson Linux 36.3.0
ROS2 Humble source build
Python 3.10.12
```

Gateway용 Python venv:

```bash
python3 -m venv ~/vla_gateway_env --system-site-packages
source ~/vla_gateway_env/bin/activate
```

SO101에는 LeRobot과 SmolVLA를 설치하지 않는다.

SO101는 다음만 수행한다.

```text
ROS2 topic 구독
이미지 resize/JPEG 압축
SensorPacket 전송
ActionPacket 수신
Safety 검사
ROS2 target 발행
```


## 제조사 ROS2 실제 Topic Audit

QuickRef 예시의 일반 토픽과 실제 장비 토픽 이름이 달랐다.

### 실제 카메라 토픽

```text
/arm/front_cam
/arm/top_cam
```

두 카메라 모두:

```text
Type: sensor_msgs/msg/Image
QoS Reliability: RELIABLE
Encoding: bgr8
Resolution: 640×480
```

확인된 물리적 카메라 역할:

```text
/arm/front_cam = 손목 카메라(Eye-in-Hand)
/arm/top_cam   = 탑뷰 카메라
```

측정 FPS:

```text
/arm/front_cam ≈ 16 Hz
/arm/top_cam   ≈ 17 Hz
```

### 실제 관절 토픽

```text
/follower/joint_states
/follower/joint_targets
/follower/safety/torque_enable
```

`/follower/joint_states`:

```text
Type: sensor_msgs/msg/JointState
Publisher: feetech_follower_driver
QoS: RELIABLE
```

`/follower/joint_targets`:

```text
Type: sensor_msgs/msg/JointState
Subscriber: feetech_follower_driver
QoS: RELIABLE
```

일반 `/joint_states` 토픽은 존재했으나 Publisher가 없었기 때문에 실제 로봇 상태로 사용하면 안 된다.

---

## 5. 확인된 Joint 이름과 순서

실제 `/follower/joint_states`의 joint 순서는 프로젝트 예상과 일치했다.

```text
shoulder_pan
shoulder_lift
elbow_flex
wrist_flex
wrist_roll
gripper
```

확인 당시 한 Sample:

```text
shoulder_pan  = -0.1058446744
shoulder_lift = -1.7518060598
elbow_flex    =  1.0676506284
wrist_flex    =  1.3591069781
wrist_roll    =  1.4726215564
gripper       = -0.0598252507
```

ROS2 관절 상태는 radian 규모로 보인다.

`shoulder_lift`가 매뉴얼 기계적 하한 `-1.75`보다 약간 작게 측정됐으므로, 매뉴얼의 기계적 범위를 엄격한 Safety 범위로 그대로 적용하면 현재 자세부터 거부될 수 있다.

실제 운용 Safety limit은 별도로 정해야 한다.

---

## 6. 확정된 Gateway 설정

`configs/so101_gateway.yaml`에서 확인된 설정:

```yaml
bridge:
  host: 127.0.0.1
  port: 49100
  gateway_id: so101_robot

ros:
  node_name: so101_sensor_gateway
  qos_depth: 5
  front_camera_topic: /arm/front_cam
  top_camera_topic: /arm/top_cam
  joint_states_topic: /follower/joint_states
  joint_targets_topic: /follower/joint_targets

sensor:
  fps: 5
  stale_timeout_s: 1.0
  width: 256
  height: 256
  output_encoding: jpeg-rgb
  jpeg_quality: 85
  instruction: Pick up the object and place it in the target bin
  joint_names:
    - shoulder_pan
    - shoulder_lift
    - elbow_flex
    - wrist_flex
    - wrist_roll
    - gripper

actions:
  actuation_enabled: false
  stale_timeout_s: 0.5
  stream_reconnect_s: 1.0
  publish_rate_limit_hz: 20

safety:
  limits_required_for_actuation: true
  max_delta_per_step: 0.05
  joint_limits: {}
```

중요:

```text
actuation_enabled=false
```

상태를 유지하고 있으므로 Dry-run에서는 실제 joint target을 발행하지 않아야 한다.

`joint_limits`는 아직 비어 있으며 실제 제어 전에 반드시 채워야 한다.

---

## 7. SO101 ROS Gateway 구독 검증

Gateway 실행 후 확인 결과:

```text
/arm/front_cam:
Publisher count: 1
Subscription count: 1

/arm/top_cam:
Publisher count: 1
Subscription count: 1

/follower/joint_states:
Publisher count: 1
Subscription count: 1
```

따라서 Gateway가 실제 카메라 2개와 관절 상태를 구독하는 것은 확인됐다.

`/follower/joint_targets`에는:

```text
Publisher count: 1
Subscription count: 1
```

이 표시됐다.

Publisher count가 1인 이유는 Gateway가 시작할 때 Publisher 객체를 생성하기 때문이다. Dry-run에서 실제 메시지가 발행됐다는 의미는 아니다.

실제 발행 여부는 다음과 같이 별도로 확인해야 한다.

```bash
timeout 10 ros2 topic echo /follower/joint_targets
```

---

## 8. A6000 연결 및 SSH Tunnel 검증

SO101에서 SSH Local Port Forwarding 사용:

```bash
ssh -N \
  -L 49100:127.0.0.1:49100 \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=10 \
  -o ServerAliveCountMax=3 \
  USER@A6000_IP
```

SO101에서 확인 결과:

```text
LISTEN 127.0.0.1:49100 users:(("ssh"...))
LISTEN [::1]:49100 users:(("ssh"...))
```

TCP 경로 검사:

```bash
nc -vz 127.0.0.1 49100
```

결과:

```text
Connection to 127.0.0.1 49100 port succeeded!
```

따라서 다음 연결은 정상이다.

```text
SO101 localhost:49100
→ SSH tunnel
→ A6000 localhost:49100
→ RemoteSO101 bridge
```

---

## 9. End-to-End Dry-run 결과

Gateway 실행:

```bash
python so101_gateway/so101_sensor_gateway.py \
  --config configs/so101_gateway.yaml \
  --dry-run
```

초기에는 카메라 첫 프레임을 기다리며 다음 로그가 출력됐다.

```text
Waiting for fresh sensors: front camera frame not received yet
```

카메라 프레임 수신 후 A6000 추론 결과가 SO101까지 돌아왔다.

대표 로그:

```text
DRY-RUN action sequence_id=1
targets=[22.27041, -16.16802, 32.30819, 36.9311, -22.99906, 2.12223]
```

이후 sequence 18까지 Action이 연속으로 수신됐다.

따라서 다음 전체 경로가 동작한 것으로 판단한다.

```text
SO101 camera/joint ROS2
→ SO101 Gateway
→ JPEG SensorPacket
→ SSH tunnel
→ A6000 bridge
→ RemoteSO101
→ Official RobotClient
→ PolicyServer
→ SmolVLA
→ ActionPacket
→ SSH tunnel
→ SO101 Gateway
```

End-to-End Dry-run 통신과 추론은 성공했다.

---

## 10. 발견된 관절 단위 불일치

ROS2 상태 값은 대략 `-2 ~ +2` 범위의 radian 값이다.

하지만 모델 Action은 다음과 같이 수십 단위다.

```text
[22.27, -16.17, 32.31, 36.93, -23.00, 2.12]
```

사용한 LeRobot SO100 데이터셋의 좌표계는 다음일 가능성이 매우 높다.

```text
앞 5개 arm joints:
degree

gripper:
0~100 calibrated range
```

필요한 Adapter:

```text
ROS2 Observation
앞 5개 radian → degree
gripper driver value → 0~100

SmolVLA Action
앞 5개 degree → radian
gripper 0~100 → driver value
```

그리퍼 방향과 open/closed 값은 실제 장비에서 별도로 측정해야 한다.

단위 Adapter 구현 전에는 실제 Action을 발행하면 안 된다.

---


## 12. 현재 상태 판정

완료:

```text
ROS2 실제 topic audit
카메라 2개 수신
관절 상태 수신
Gateway ROS subscription
이미지 resize/JPEG
SSH tunnel
A6000 bridge 연결
SmolVLA 추론
ActionPacket SO101 반환
End-to-End Dry-run
```

미완료:

```text
관절 radian ↔ degree Adapter
gripper driver 값 ↔ 0~100 Adapter
Dry-run Safety validation
최종 joint limits
Heartbeat/Watchdog 안정화
간헐적 DEADLINE_EXCEEDED 해결
Ctrl+C shutdown 오류 수정
실제 joint target 미발행 확인
실물 Hold test
실물 미세 관절 이동
SmolVLA 실제 제어
```

현재는 End-to-End Dry-run은 성공했으나 실물 제어 가능한 단계는 아니다.

`actuation_enabled`는 계속 `false`로 유지해야 한다.

---

## 13. 다음 구현 우선순위

1. 앞 5개 radian↔degree 변환
2. gripper 변환
3. joint limits 설정
4. Dry-run Safety 검사
5. 변환된 Action Dry-run
6. 현재 위치 Hold
7. 단일 관절 0.005 rad
8. 단일 관절 0.01 rad
9. 이후 제한된 SmolVLA 실물 시험