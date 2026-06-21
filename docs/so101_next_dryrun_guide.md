# SO101 Next Dry-run Guide

다음 dry-run의 목적은 **실물 publish 없이 SmolVLA action이 SO101 ROS 단위로 안전하게 변환되는지 확정**하는 것이다.

이번 dry-run에서 확인해야 하는 것:

- SO101 실제 topic 연결
- A6000 unit adapter 동작
- arm action `degree -> radian` 변환
- gripper action `0~33 -> 0.00~0.80 rad` 변환
- SO101 Gateway safety validation 동작
- dry-run에서 `/follower/joint_targets` 미발행
- SmolVLA action이 실제 publish 가능한 크기인지, 아니면 safety가 reject하는지

---

## 1. 이번 dry-run의 정상 판정

정상은 반드시 `DRY-RUN valid action`만 의미하지 않는다.

다음 둘 다 정상이다.

```text
DRY-RUN valid action ...
```

또는

```text
Rejected action sequence_id=...: joint delta ...
```

`Rejected action`은 action이 너무 커서 safety가 막았다는 뜻이다. 다음 실물 단계로 넘어가기 전에는 이 reject 이유를 보고 `max_delta_per_step`, action smoothing, hold test 계획을 정해야 한다.

절대 나오면 안 되는 것:

```text
Published action sequence_id=...
```

이번 dry-run에서는 `/follower/joint_targets`에 메시지가 발행되면 안 된다.

---

## 2. SO101 Topic 기준

실제 장비 기준 topic:

```text
/arm/front_cam
/arm/top_cam
/follower/joint_states
/follower/joint_targets
/follower/safety/torque_enable
```

카메라:

```text
/arm/front_cam = wrist / eye-in-hand camera
/arm/top_cam   = top-view camera
encoding       = bgr8
resolution     = 640x480
```

Joint order:

```text
shoulder_pan
shoulder_lift
elbow_flex
wrist_flex
wrist_roll
gripper
```

SO101 ROS joint target 단위:

```text
absolute radian position target
```

---

## 3. Unit Mapping 기준

Arm joints:

```text
SO101 ROS observation: radian
SmolVLA policy observation/action: degree
```

변환:

```text
observation: radian -> degree
action:      degree -> radian
```

Gripper:

```text
SO101 ROS hard range: -0.174533 ~ 1.74533 rad
SO101 IK default closed: 0.00 rad
SO101 IK default open:   0.80 rad
SmolVLA checkpoint gripper action range: 0 ~ 33
```

초기 mapping:

```text
policy 0  -> ROS 0.00 rad
policy 33 -> ROS 0.80 rad
```

---

## 4. A6000 실행

### Terminal A: PolicyServer

```bash
conda activate vla-lerobot
cd ~/VLA

python policy/run_policy_server.py \
  --config configs/policy_server.smolvla.yaml
```

정상 기준:

```text
PolicyServer listening on 0.0.0.0:8080
```

또는 `8080` listen 상태가 확인되면 된다.

```bash
ss -ltnp | grep 8080
```

### Terminal B: RemoteSO101 RobotClient + bridge

```bash
conda activate vla-lerobot
cd ~/VLA

python remote_so101/run_robot_client.py \
  --config configs/remote_so101.yaml \
  --duration-s 120 \
  --max-steps 600 \
  --stop-after-actions 20 \
  --verbose
```

정상 초기 출력:

```text
bridge=127.0.0.1:49100
policy_server=127.0.0.1:8080
checkpoint=checkpoints/smolvla_so101
robot_type=remote_so101
actions_per_chunk=5
SO101 bridge server started at 127.0.0.1:49100
```

A6000 bridge listen 확인:

```bash
ss -ltnp | grep 49100
```

정상 기준:

```text
LISTEN ... 127.0.0.1:49100
```

---

## 5. SO101 실행

### Terminal 1: 제조사 bringup

```bash
source ~/physicai_arm_ws/install/setup.bash
ros2 launch physicai_arm bringup.launch.py
```

정상 확인:

```bash
ros2 topic list | grep -E "/arm/front_cam|/arm/top_cam|/follower/joint_states|/follower/joint_targets"
```

기대 출력:

```text
/arm/front_cam
/arm/top_cam
/follower/joint_states
/follower/joint_targets
```

### Terminal 2: sensor topic 확인

```bash
source ~/physicai_arm_ws/install/setup.bash

ros2 topic hz /arm/front_cam
ros2 topic hz /arm/top_cam
ros2 topic echo /follower/joint_states --once
```

정상 기준:

```text
/arm/front_cam: 약 16Hz 전후
/arm/top_cam: 약 17Hz 전후
/follower/joint_states.name: 6개 joint 이름
/follower/joint_states.position: radian 규모 값
```

### Terminal 3: SSH tunnel

```bash
ssh -N \
  -L 49100:127.0.0.1:49100 \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=10 \
  -o ServerAliveCountMax=3 \
  <A6000_USER>@<A6000_IP>
```

다른 SO101 터미널에서 확인:

```bash
ss -ltnp | grep 49100
nc -vz 127.0.0.1 49100
```

정상 기준:

```text
LISTEN 127.0.0.1:49100 users:(("ssh"...))
Connection to 127.0.0.1 49100 port succeeded!
```

### Terminal 4: dry-run 중 target 미발행 감시

```bash
source ~/physicai_arm_ws/install/setup.bash
timeout 120 ros2 topic echo /follower/joint_targets
```

정상 기준:

```text
아무 메시지도 출력되지 않음
```

이번 dry-run에서 이 터미널에 메시지가 나오면 즉시 gateway를 중단한다.

### Terminal 5: SO101 Gateway dry-run

```bash
source ~/physicai_arm_ws/install/setup.bash
source ~/vla_gateway_env/bin/activate
cd ~/VLA

python so101_gateway/so101_sensor_gateway.py \
  --config configs/so101_gateway.yaml \
  --dry-run
```

정상 초기 출력:

```text
SO101 gateway started: bridge=127.0.0.1:49100 actuation_enabled=False
```

카메라 첫 프레임이 늦으면 일시적으로 다음 로그가 나올 수 있다.

```text
Waiting for fresh sensors: front camera frame not received yet
```

카메라가 들어오면 이 로그는 멈춰야 한다.

---

## 6. Dry-run 결과 해석

### Case A: safety 통과

```text
DRY-RUN valid action sequence_id=... targets=[...] max_delta=...
```

확인할 것:

- `targets`는 모두 radian 규모여야 한다.
- arm 값은 대략 `-3.14 ~ 3.14` 안쪽이어야 한다.
- gripper는 `0.00 ~ 0.80` 근처여야 한다.
- `max_delta`가 `0.01` 이하이면 현재 config 기준으로 실물 후보가 된다.

### Case B: safety reject

```text
Rejected action sequence_id=...: joint delta for shoulder_pan=... exceeds 0.010000
```

의미:

- A6000 변환은 되었지만 현재 자세 대비 목표가 너무 크다.
- safety가 제대로 막았다.
- 바로 실물 publish로 넘어가면 안 된다.
- action smoothing, hold-first, 작은 이동 테스트가 필요하다.

### Case C: limit reject

```text
Rejected action sequence_id=...: joint target for ... outside [...]
```

의미:

- 변환된 action이 SO101 joint hard limit 밖이다.
- 해당 action은 절대 publish하면 안 된다.

### Case D: stale reject

```text
Rejected action sequence_id=...: stale action packet older than ...
```

의미:

- action stream 지연 또는 clock/timestamp 처리 문제다.
- SSH tunnel 상태와 gateway 로그를 같이 확인한다.

---

## 7. 이번 dry-run에서 기록할 것

아래 값을 `direction.md`에 기록한다.

```text
날짜/시간:
A6000 checkpoint:
PolicyServer command:
RobotClient command:
SO101 gateway command:

/arm/front_cam hz:
/arm/top_cam hz:
/follower/joint_states sample:

첫 5개 ActionPacket:
각 action의 targets:
각 action의 max_delta:
valid/rejected 여부:
reject reason:

/follower/joint_targets echo 결과:
```

---

## 8. 다음 단계 판단 기준

다음 단계로 넘어갈 수 있는 조건:

```text
1. /follower/joint_targets 미발행 확인
2. A6000 action이 ROS radian target으로 변환됨
3. gripper target이 0.00~0.80 rad 범위로 변환됨
4. safety validation 로그가 정상 동작함
5. reject가 발생하면 이유가 명확함
```

그 다음 단계는 policy action publish가 아니다.

다음 순서:

```text
1. current-position HOLD publish
2. 단일 joint 0.005 rad 이동
3. 단일 joint 0.01 rad 이동
4. gripper 0.00 -> 0.80 방향 확인
5. 제한 workspace에서 SmolVLA action publish
```

