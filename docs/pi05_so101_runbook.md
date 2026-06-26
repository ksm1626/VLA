# Pi0.5 Fine-tuned Model SO101 실행 절차

이 문서는 Pi0.5 fine-tuning이 끝난 뒤, A6000과 SO101 PC에서 Pi0.5 policy를 dry-run으로 먼저 검증하고 이후 실제 로봇 구동까지 진행하는 절차를 정리한다.

기본 원칙은 다음 순서다.

```text
1. A6000에서 fine-tuned checkpoint 확인
2. A6000에서 Pi0.5 checkpoint offline validation
3. A6000에서 Pi0.5 PolicyServer 실행
4. A6000에서 SSH tunnel 실행
5. SO101 PC에서 제조사 bringup 실행
6. SO101 PC에서 gateway dry-run 실행
7. A6000에서 RobotClient dry-run 실행
8. SO101 PC에서 actuation enabled gateway 실행
9. A6000에서 RobotClient 실제 구동 실행
```

---

## 0. 전제

Pi0.5 fine-tuning이 완료되어 있어야 한다.

현재 학습 config 기준 출력 경로는 다음 계열이다.

```text
outputs/train/pi05_so101_v1/checkpoints/<STEP>/pretrained_model
```

학습을 `20,000 steps`로 끝냈다면 예상 checkpoint는 다음이다.

```text
outputs/train/pi05_so101_v1/checkpoints/020000/pretrained_model
```

만약 중간 checkpoint만 사용한다면 예시는 다음과 같다.

```text
outputs/train/pi05_so101_v1/checkpoints/005000/pretrained_model
```

아래 문서에서는 `020000`을 최종 checkpoint로 가정한다. 실제 생성된 step 번호가 다르면 해당 부분만 바꿔서 사용한다.

---

## 1. A6000: Checkpoint 확인

A6000에서 실행한다.

```bash
conda activate vla-lerobot
cd ~/VLA

ls outputs/train/pi05_so101_v1/checkpoints
ls outputs/train/pi05_so101_v1/checkpoints/020000/pretrained_model
```

정상 기준:

```text
config.json
model.safetensors
policy_preprocessor.json
policy_postprocessor.json
train_config.json
```

`020000`이 없고 `005000`, `010000` 같은 폴더만 있으면 존재하는 가장 최신 checkpoint를 사용한다.

---

## 2. A6000: Pi0.5 Fine-tuned Checkpoint Offline Validation

실물 로봇을 움직이기 전에 dataset sample 하나로 action chunk가 정상 생성되는지 확인한다.

```bash
CUDA_VISIBLE_DEVICES=0 PYTHONNOUSERSITE=1 python policy/validate_checkpoint.py \
  --checkpoint outputs/train/pi05_so101_v1/checkpoints/020000/pretrained_model \
  --device cuda \
  --actions-per-chunk 5
```

정상 출력 기준:

```text
policy_type=pi05
policy_device=cuda
dataset_frames=40635
raw_action_chunk_shape=(1, 5, 6) finite=True
postprocessed_action_chunk_shape=(5, 6) finite=True
checkpoint validation passed.
```

이 단계가 실패하면 PolicyServer나 SO101 gateway를 켜지 않는다.

---

## 3. A6000: Pi0.5 Runtime Config Checkpoint 경로 수정

`configs/remote_so101.pi05.yaml`에서 checkpoint 경로를 fine-tuned checkpoint로 맞춘다.

```yaml
policy:
  policy_type: pi05
  pretrained_name_or_path: outputs/train/pi05_so101_v1/checkpoints/020000/pretrained_model
```

`configs/policy_server.pi05.yaml`의 `client_handshake_defaults`도 문서용 기본값이므로 같은 checkpoint로 맞춰두면 헷갈리지 않는다.

```yaml
client_handshake_defaults:
  policy_type: pi05
  pretrained_name_or_path: outputs/train/pi05_so101_v1/checkpoints/020000/pretrained_model
```

주의: LeRobot PolicyServer는 시작 시 checkpoint를 직접 고르지 않고, RobotClient handshake에서 policy 정보를 받는다. 실제로 중요한 것은 `configs/remote_so101.pi05.yaml`의 `policy.pretrained_name_or_path`다.

---

## 4. A6000: PolicyServer Dry-run

먼저 실행 명령만 확인한다.

```bash
python policy/run_policy_server.py \
  --config configs/policy_server.pi05.yaml \
  --dry-run
```

정상 출력 기준:

```text
python -m lerobot.async_inference.policy_server --host 0.0.0.0 --port 8080
```

---

## 5. A6000: PolicyServer 실행

A6000 터미널 1에서 실행한다.

```bash
conda activate vla-lerobot
cd ~/VLA

CUDA_VISIBLE_DEVICES=0 PYTHONNOUSERSITE=1 python policy/run_policy_server.py \
  --config configs/policy_server.pi05.yaml
```

정상 기준:

```text
PolicyServer started
Listening on 0.0.0.0:8080
```

정확한 로그 문구는 LeRobot 버전에 따라 조금 다를 수 있다. 핵심은 에러 없이 `8080` port에서 대기 상태가 되는 것이다.

---

## 6. A6000: SSH Tunnel 실행

A6000 터미널 2에서 실행한다.

SO101 PC에서 `127.0.0.1:49100`으로 접속하면 A6000의 RemoteSO101 bridge로 연결되도록 tunnel을 연다.

예시:

```bash
ssh -N \
  -L 49100:127.0.0.1:49100 \
  -p <SSH_PORT> \
  -o ExitOnForwardFailure=yes \
  -o ServerAliveInterval=10 \
  -o ServerAliveCountMax=3 \
  <USER>@<A6000_PUBLIC_IP>
```

프로젝트에서 사용 중인 `a6000.sh`가 있으면 SO101 PC에서 그 스크립트를 실행해도 된다.

정상 기준:

```text
터미널이 아무 출력 없이 멈춰 있음
```

이 상태가 정상이다. 이 터미널은 닫지 않는다.

---

## 7. SO101 PC: 제조사 ROS2 Bringup 실행

SO101 PC 터미널 1에서 실행한다.

```bash
source ~/physicai_arm_ws/install/setup.bash
ros2 launch physicai_arm bringup.launch.py
```

정상 기준:

```text
/front 또는 /arm camera node 관련 에러 없음
joint state publish 관련 에러 없음
```

다른 터미널에서 topic을 확인한다.

```bash
source ~/physicai_arm_ws/install/setup.bash
ros2 topic list
```

정상 기준으로 아래 topic이 보여야 한다.

```text
/arm/front_cam
/arm/top_cam
/follower/joint_states
/follower/joint_targets
```

카메라 hz 확인:

```bash
ros2 topic hz /arm/front_cam
ros2 topic hz /arm/top_cam
```

joint state 확인:

```bash
ros2 topic hz /follower/joint_states
```

---

## 8. SO101 PC: Gateway Dry-run 실행

SO101 PC 터미널 2에서 실행한다.

```bash
source ~/physicai_arm_ws/install/setup.bash
source ~/vla_gateway_env/bin/activate
cd ~/Desktop/VLA

python so101_gateway/so101_sensor_gateway.py \
  --config configs/so101_gateway.yaml \
  --dry-run
```

정상 출력 기준:

```text
SO101 gateway started: bridge=127.0.0.1:49100 actuation_enabled=True
```

Dry-run에서는 action을 받아도 `/follower/joint_targets`로 실제 publish하지 않는다.

정상적으로 센서가 들어오면 다음 문제가 없어야 한다.

```text
Waiting for fresh sensors: front camera frame not received yet
Waiting for fresh sensors: top camera frame not received yet
```

위 로그가 계속 반복되면 camera topic이 안 들어오는 상태다. 이 경우 bringup과 카메라 topic부터 다시 확인한다.

---

## 9. A6000: RobotClient Dry-run 실행

A6000 터미널 3에서 실행한다.

```bash
conda activate vla-lerobot
cd ~/VLA

PYTHONNOUSERSITE=1 python remote_so101/run_robot_client.py \
  --config configs/remote_so101.pi05.yaml \
  --dry-run
```

정상 출력 기준:

```text
bridge=127.0.0.1:49100
policy_server=127.0.0.1:8080
policy_type=pi05
checkpoint=outputs/train/pi05_so101_v1/checkpoints/020000/pretrained_model
robot_type=remote_so101
actions_per_chunk=50
```

여기서 checkpoint가 `005000`으로 나오면 config가 아직 옛 경로를 보고 있는 것이다. `configs/remote_so101.pi05.yaml`을 수정한다.

---

## 10. A6000: RobotClient Bounded Dry-run 실행

이 단계는 SO101 gateway가 `--dry-run` 상태일 때 실행한다. 즉 action chunk가 왕복되는지 확인하지만 로봇은 실제로 움직이지 않는다.

```bash
PYTHONNOUSERSITE=1 python remote_so101/run_robot_client.py \
  --config configs/remote_so101.pi05.yaml \
  --duration-s 60 \
  --max-steps 600 \
  --stop-after-actions 10
```

정상 출력 기준:

A6000 RobotClient 쪽:

```text
SO101 bridge server started at 127.0.0.1:49100
policy_type=pi05
checkpoint=outputs/train/pi05_so101_v1/checkpoints/020000/pretrained_model
RobotClient bounded run complete: observations=<N> actions=<M>
```

`actions=<M>`이 1 이상이면 PolicyServer에서 action을 받아온 것이다.

PolicyServer 쪽:

```text
Action chunk generated
Preprocessing and inference took ...s
action shape: torch.Size([1, 50, 6])
```

SO101 gateway dry-run 쪽:

```text
DRY-RUN action ...
```

또는 action을 받았지만 publish하지 않았다는 로그가 나와야 한다.

---

## 11. 실제 구동 전 확인

실제 로봇을 움직이기 전에 아래를 확인한다.

`configs/so101_gateway.yaml`:

```yaml
actions:
  actuation_enabled: true
  clamp_to_joint_limits: true
  publish_rate_limit_hz: 0

safety:
  limits_required_for_actuation: true
  max_delta_per_step: null
```

현재 설정 의미:

```text
actuation_enabled=true: 실제 /follower/joint_targets publish 허용
clamp_to_joint_limits=true: joint limit 초과 action은 limit 안으로 clamp
publish_rate_limit_hz=0: 별도 publish rate 제한 없음
max_delta_per_step=null: per-step delta limit 비활성화
```

처음 실제 구동은 짧게 제한해서 실행한다.

추천 시작값:

```text
duration-s: 30~60
max-steps: 300~600
stop-after-actions: 3~10
```

---

## 12. SO101 PC: Gateway 실제 구동 실행

SO101 PC에서 dry-run 옵션을 빼고 gateway를 실행한다.

SO101 PC 터미널 2:

```bash
source ~/physicai_arm_ws/install/setup.bash
source ~/vla_gateway_env/bin/activate
cd ~/Desktop/VLA

python so101_gateway/so101_sensor_gateway.py \
  --config configs/so101_gateway.yaml
```

정상 출력 기준:

```text
SO101 gateway started: bridge=127.0.0.1:49100 actuation_enabled=True
```

이 상태에서는 A6000에서 action이 오면 safety filter 통과 후 `/follower/joint_targets`로 실제 publish된다.

---

## 13. A6000: Pi0.5 실제 짧은 구동

A6000 터미널 3에서 실행한다.

처음에는 짧게 실행한다.

```bash
PYTHONNOUSERSITE=1 python remote_so101/run_robot_client.py \
  --config configs/remote_so101.pi05.yaml \
  --duration-s 30 \
  --max-steps 300 \
  --stop-after-actions 3
```

정상 출력 기준:

```text
SO101 bridge server started at 127.0.0.1:49100
policy_type=pi05
checkpoint=outputs/train/pi05_so101_v1/checkpoints/020000/pretrained_model
RobotClient bounded run complete: observations=<N> actions=3
```

SO101 gateway 쪽에서는 action publish 로그가 나와야 한다.

로봇이 예상 밖으로 움직이면 즉시 중단한다.

중단 방법:

```text
A6000 RobotClient 터미널에서 Ctrl+C
SO101 gateway 터미널에서 Ctrl+C
필요하면 제조사 bringup 종료
```

---

## 14. 조금 더 긴 실제 구동

짧은 구동이 안전하면 action 수를 늘린다.

```bash
PYTHONNOUSERSITE=1 python remote_so101/run_robot_client.py \
  --config configs/remote_so101.pi05.yaml \
  --duration-s 60 \
  --max-steps 600 \
  --stop-after-actions 20
```

더 길게 제한 없이 실행하고 싶으면 `--stop-after-actions`를 빼거나 config의 `runtime.stop_after_actions: 0`을 사용한다.

하지만 초기 Pi0.5 실물 테스트에서는 제한 없이 실행하는 것을 권장하지 않는다. 먼저 `3`, `10`, `20` action 단위로 동작을 확인하는 것이 안전하다.

---

## 15. FPS / Chunk / Aggregation 조절 위치

Pi0.5 runtime 조절은 `configs/remote_so101.pi05.yaml`에서 한다.

```yaml
policy:
  actions_per_chunk: 50
  chunk_size_threshold: 0.6
  aggregate_fn_name: weighted_average

runtime:
  fps: 10
```

SO101 gateway sensor 전송 FPS는 `configs/so101_gateway.yaml`에서 한다.

```yaml
sensor:
  fps: 10
```

권장 실험 순서:

```text
1. fps=10, actions_per_chunk=50, weighted_average
2. fps=20, actions_per_chunk=50, weighted_average
3. fps=30, actions_per_chunk=50, weighted_average
4. 필요 시 actions_per_chunk=30 또는 40 비교
```

주의:

```text
fps가 높으면 반응은 빨라지지만 정확도와 안정성이 떨어질 수 있다.
chunk가 크면 긴 trajectory를 한 번에 받지만 새 observation 반영이 느려질 수 있다.
weighted_average는 overwrite보다 부드럽지만 반응이 둔해질 수 있다.
```

---

## 16. 문제 발생 시 빠른 체크

### PolicyServer 연결 실패

A6000에서 확인:

```bash
ss -ltnp | grep 8080
```

### Bridge 연결 실패

A6000 RobotClient가 bridge server를 열었는지 확인한다.

```text
SO101 bridge server started at 127.0.0.1:49100
```

SO101 gateway가 SSH tunnel을 통해 `127.0.0.1:49100`에 붙는 구조인지 확인한다.

### Sensor packet timeout

SO101에서 확인:

```bash
ros2 topic list
ros2 topic hz /arm/front_cam
ros2 topic hz /arm/top_cam
ros2 topic hz /follower/joint_states
```

### Action이 전부 reject됨

SO101 gateway log에서 reject 이유를 확인한다.

주요 원인:

```text
joint limit 초과
joint name mismatch
stale action
NaN/Inf
sensor stale
```

### 로봇이 너무 빠르거나 부정확함

`configs/remote_so101.pi05.yaml`에서 조절한다.

```yaml
runtime:
  fps: 10

policy:
  actions_per_chunk: 50
  aggregate_fn_name: weighted_average
```

처음에는 fps를 낮추고 짧은 action count로 테스트한다.
