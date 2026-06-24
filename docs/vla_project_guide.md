# VLA Project Integrated Guide

이 문서는 VLA 프로젝트의 전체 실행 흐름을 한 번에 보기 위한 통합 가이드다. 세부 troubleshooting은 각 전용 문서를 참고한다.

상세 문서:

- [SO101 Gateway Runbook](./so101_gateway_runbook.md)
- [SO101 Next Dry-run Guide](./so101_next_dryrun_guide.md)
- [SO101 Teleop Dataset Recording Guide](./so101_teleop_record.md)

---

## 1. 프로젝트 목표

v1 목표는 **SmolVLA를 fine-tuning하고, A6000에서 LeRobot 공식 async 추론 서버를 실행한 뒤, SO101 Robot PC의 ROS2 gateway를 통해 실물 로봇을 안전하게 제어하는 파이프라인 검증**이다.

핵심 성공 기준:

- A6000에서 SmolVLA fine-tuning 가능
- A6000에서 fine-tuned checkpoint 검증 가능
- A6000에서 LeRobot `PolicyServer` 실행 가능
- A6000에서 `RemoteSO101Robot` + LeRobot `RobotClient` 실행 가능
- SO101 PC에서 제조사 ROS2 bringup 유지
- SO101 PC에서 gateway dry-run으로 action path 검증
- 실물 action은 SO101-side safety gate를 통과한 경우에만 publish
- teleop native dataset 수집, replay, LeRobotDataset 변환 가능

---

## 2. 전체 아키텍처

```text
SO101 Robot PC
  제조사 ROS2 Humble bringup
  /arm/front_cam
  /arm/top_cam
  /follower/joint_states
  /follower/joint_targets
        |
        v
  so101_gateway/
    so101_sensor_gateway.py
    record_so101_episode.py
    replay_so101_episode.py
        |
        | custom gRPC over SSH tunnel
        v
A6000 Server
  remote_so101/
    RemoteSO101Robot
    gRPC bridge server
        |
        v
  LeRobot RobotClient
        |
        v
  LeRobot PolicyServer
        |
        v
  fine-tuned SmolVLA checkpoint
```

중요한 분리 원칙:

- SO101 PC는 ROS2/rclpy, OpenCV, numpy 중심이다.
- SO101 PC에는 LeRobot, torch, transformers를 설치하지 않는다.
- A6000은 LeRobot/SmolVLA Python 3.12 환경이다.
- A6000은 v1 runtime에서 ROS2 topic을 직접 구독하지 않는다.
- SO101과 A6000 사이 로봇 sensor/action 전달만 custom gRPC를 쓴다.

---

## 3. 주요 디렉토리

```text
configs/          실행 설정 파일
training/         SmolVLA fine-tuning wrapper
policy/           PolicyServer 및 checkpoint validation
remote_so101/     A6000-side RemoteSO101Robot, gRPC bridge
so101_gateway/    SO101-side ROS2 gateway, record, replay
recording/        native dataset validation, LeRobotDataset 변환
proto/            SO101-A6000 custom gRPC schema
docs/             runbook 및 가이드 문서
tests/            ROS 없이 돌 수 있는 단위 테스트
datasets/         local native/lerobot dataset 위치
outputs/          training output
checkpoints/      checkpoint symlink
```

주요 config:

| Config | 용도 |
|---|---|
| `configs/finetune.smolvla.yaml` | SmolVLA fine-tuning |
| `configs/policy_server.smolvla.yaml` | LeRobot PolicyServer |
| `configs/remote_so101.yaml` | A6000 RobotClient + RemoteSO101 |
| `configs/so101_gateway.yaml` | SO101 실물 gateway |
| `configs/so101_recording.yaml` | SO101 teleop recording |
| `configs/mock_gateway.so101.yaml` | A6000-only mock gateway |

---

## 4. 안전 기본값

실물 로봇 관련 기본값은 보수적으로 둔다.

```yaml
actions:
  actuation_enabled: false
```

이 값은 [configs/so101_gateway.yaml](../configs/so101_gateway.yaml)의 기본값이다.

실물 publish가 가능한 경우:

- gateway는 `actions.actuation_enabled: true`
- replay는 `--actuate`와 `actions.actuation_enabled: true`가 모두 필요
- SmolVLA gateway dry-run에서는 `/follower/joint_targets`가 publish되면 안 됨
- safety filter가 NaN/Inf, joint order, joint limit, max delta를 검사

금지:

- SO101에서 `apt upgrade`, `torch`, `numpy`, `opencv` 임의 업그레이드
- 데이터 파일을 GitHub에 push
- 개인 IP, 포트, 계정, token을 공개 문서나 tracked script에 기록
- 검증 없이 `actuation_enabled=true`로 장시간 실행

---

## 5. A6000 환경 준비

LeRobot 환경은 `vla-lerobot` conda 환경을 기준으로 한다.

```bash
conda activate vla-lerobot
cd ~/VLA
python training/validate_lerobot_env.py
```

환경을 새로 만들 때:

```bash
bash scripts/setup_lerobot_env.sh
```

정상 기준:

- `lerobot` import OK
- `lerobot.async_inference.policy_server` import OK
- `lerobot.async_inference.robot_client` import OK
- `lerobot-train` command 존재
- CUDA는 가능하면 OK, 없으면 warning일 수 있음

---

## 6. SmolVLA Fine-tuning

기본 fine-tuning 설정은 [configs/finetune.smolvla.yaml](../configs/finetune.smolvla.yaml)이다.

현재 기본값:

```yaml
policy:
  path: lerobot/smolvla_base
dataset:
  repo_id: lerobot/svla_so101_pickplace
training:
  output_dir: outputs/train/smolvla_so101_2
  steps: 20000
```

명령 확인:

```bash
conda activate vla-lerobot
cd ~/VLA
CUDA_VISIBLE_DEVICES=2 python training/run_finetune.py \
  --config configs/finetune.smolvla.yaml \
  --dry-run
```

실제 학습:

```bash
CUDA_VISIBLE_DEVICES=2 python training/run_finetune.py \
  --config configs/finetune.smolvla.yaml
```

local LeRobotDataset으로 학습할 때는 config를 다음처럼 바꾼다.

```yaml
dataset:
  repo_id: null
  root: datasets/lerobot/so101_pickplace_v1
```

checkpoint symlink는 추론 config가 기대하는 경로로 맞춘다.

```text
checkpoints/smolvla_so101
```

---

## 7. Checkpoint 검증

fine-tuned checkpoint가 load 되고 sample inference가 되는지 확인한다.

```bash
conda activate vla-lerobot
cd ~/VLA
python policy/validate_checkpoint.py \
  --config configs/policy_server.smolvla.yaml
```

확인할 것:

- checkpoint path 존재
- dataset sample load 가능
- action shape가 6D joint action과 맞음
- NaN/Inf 없음
- camera key mapping이 checkpoint와 맞음

---

## 8. PolicyServer 실행

A6000 Terminal A:

```bash
conda activate vla-lerobot
cd ~/VLA
python policy/run_policy_server.py \
  --config configs/policy_server.smolvla.yaml
```

dry-run:

```bash
python policy/run_policy_server.py \
  --config configs/policy_server.smolvla.yaml \
  --dry-run
```

기본 listen:

```text
0.0.0.0:8080
```

---

## 9. A6000 RemoteSO101 RobotClient

A6000 Terminal B:

```bash
conda activate vla-lerobot
cd ~/VLA
python remote_so101/run_robot_client.py \
  --config configs/remote_so101.yaml
```

dry-run:

```bash
python remote_so101/run_robot_client.py \
  --config configs/remote_so101.yaml \
  --dry-run
```

RobotClient 역할:

- A6000 `127.0.0.1:49100`에서 SO101 bridge gRPC server 실행
- SO101 gateway가 보낸 sensor packet을 `RemoteSO101Robot.get_observation()`으로 변환
- LeRobot `RobotClient`로 PolicyServer에 observation 전송
- 받은 action chunk를 SO101 gateway로 stream

---

## 10. SO101 준비

SO101에서는 제조사 ROS2 workspace와 lightweight gateway env를 사용한다.

```bash
source ~/physicai_arm_ws/install/setup.bash
source ~/vla_gateway_env/bin/activate
cd ~/Desktop/VLA
```

필수 topic 확인:

```bash
ros2 topic list | grep -E "/arm/front_cam|/arm/top_cam|/follower/joint_states|/follower/joint_targets"
ros2 topic hz /arm/front_cam
ros2 topic hz /arm/top_cam
ros2 topic echo /follower/joint_states --once
```

카메라가 죽었을 때:

```bash
sudo systemctl restart nvargus-daemon
```

자세한 SO101 준비는 [SO101 Gateway Runbook](./so101_gateway_runbook.md)을 따른다.

---

## 11. SSH Tunnel

SO101에서 A6000 bridge로 연결되는 local tunnel을 연다.

```bash
ssh -N \
  -L 49100:127.0.0.1:49100 \
  -p <SSH_PORT> \
  <A6000_USER>@<A6000_HOST>
```

주의:

- 실제 IP, 계정, 포트는 tracked 파일에 넣지 않는다.
- `a6000.sh` 같은 개인 스크립트는 `.gitignore`에 둔다.
- tunnel이 끊기면 action path도 끊긴다.

---

## 12. SO101 Gateway Dry-run

SO101 Terminal:

```bash
source ~/physicai_arm_ws/install/setup.bash
source ~/vla_gateway_env/bin/activate
cd ~/Desktop/VLA

python so101_gateway/so101_sensor_gateway.py \
  --config configs/so101_gateway.yaml \
  --dry-run
```

정상 예:

```text
SO101 gateway started: bridge=127.0.0.1:49100 actuation_enabled=False
DRY-RUN valid action sequence_id=...
```

dry-run에서는 `/follower/joint_targets`가 publish되면 안 된다.

감시:

```bash
timeout 120 ros2 topic echo /follower/joint_targets
```

자세한 dry-run 판정은 [SO101 Next Dry-run Guide](./so101_next_dryrun_guide.md)를 참고한다.

---

## 13. 실물 Actuation

실물 publish는 dry-run이 정상이고 로봇 주변이 안전할 때만 한다.

실행 조건:

- A6000 PolicyServer 실행 중
- A6000 RobotClient 실행 중
- SSH tunnel 연결됨
- SO101 bringup 정상
- SO101 gateway dry-run에서 valid action 확인
- `/follower/joint_targets`가 absolute radian target임을 확인
- `configs/so101_gateway.yaml`의 `actions.actuation_enabled: true`

SO101:

```bash
python so101_gateway/so101_sensor_gateway.py \
  --config configs/so101_gateway.yaml
```

실물 테스트는 작은 workspace, 낮은 속도, 짧은 시간으로 시작한다. 테스트 후에는 config를 다시 `actuation_enabled: false`로 되돌린다.

---

## 14. Teleop 방식

학습용 action은 항상 `/follower/joint_targets`를 기준으로 저장한다.

Leader arm:

```bash
source ~/physicai_arm_ws/install/setup.bash
ros2 topic echo /leader/joint_states --once

ros2 run physicai_arm teleoperation --ros-args \
  -p input_topic:=/leader/joint_states \
  -p output_topic:=/follower/joint_targets
```

Joystick:

```bash
source ~/physicai_arm_ws/install/setup.bash
ros2 run joy joy_node --ros-args -p device:=/dev/input/js0
```

```bash
source ~/physicai_arm_ws/install/setup.bash
ros2 run physicai_arm joy_to_target
```

```bash
source ~/physicai_arm_ws/install/setup.bash
ros2 run physicai_arm ik_calc --ros-args \
  -p joint_topic:=/follower/joint_states \
  -p output_topic:=/follower/joint_targets
```

확인:

```bash
ros2 topic echo /follower/joint_targets --once
```

---

## 15. Native Dataset Recording

SO101에서 native dataset을 MP4 + JSONL로 기록한다.

```bash
source ~/physicai_arm_ws/install/setup.bash
source ~/vla_gateway_env/bin/activate
cd ~/Desktop/VLA

python so101_gateway/record_so101_episode.py \
  --config configs/so101_recording.yaml \
  --task "Pick up the cube and place it in the target bin" \
  --teleop-source leader
```

조이스틱이면:

```bash
python so101_gateway/record_so101_episode.py \
  --config configs/so101_recording.yaml \
  --task "Pick up the cube and place it in the target bin" \
  --teleop-source joystick
```

record 중 조작:

| 입력 | 동작 |
|---|---|
| `Enter` | 현재 episode 저장 후 다음 episode 시작 |
| `c` + `Enter` | 현재 episode discard 후 다음 episode 시작 |
| `q` + `Enter` | 현재 episode 저장 후 종료 |
| `Ctrl+C` | 현재 episode 저장 후 종료 |

저장 위치:

```text
/home/soda/Desktop/VLA/datasets/native/so101_pickplace_v1/
  dataset.yaml
  episodes/
    episode_000001/
      episode.json
      frames.jsonl
      videos/front.mp4
      videos/top.mp4
```

---

## 16. Native Dataset 검증

SO101 또는 A6000에서 검증한다.

```bash
python recording/validate_native_dataset.py \
  --root /home/soda/Desktop/VLA/datasets/native/so101_pickplace_v1
```

확인 항목:

- `dataset.yaml`
- `episode.json`
- `frames.jsonl`
- `videos/front.mp4`
- `videos/top.mp4`
- frame count 일치
- joint order 일치
- state/action 6D
- NaN/Inf 없음

---

## 17. Episode Replay

record가 실제 trajectory로 재현 가능한지 SO101에서 검증한다.

기본 dry-run:

```bash
source ~/physicai_arm_ws/install/setup.bash
source ~/vla_gateway_env/bin/activate
cd ~/Desktop/VLA

python so101_gateway/replay_so101_episode.py \
  --episode /home/soda/Desktop/VLA/datasets/native/so101_pickplace_v1/episodes/episode_000001 \
  --config configs/so101_gateway.yaml
```

실제 replay는 두 조건이 모두 필요하다.

- `configs/so101_gateway.yaml`의 `actions.actuation_enabled: true`
- 명령어에 `--actuate`

```bash
python so101_gateway/replay_so101_episode.py \
  --episode /home/soda/Desktop/VLA/datasets/native/so101_pickplace_v1/episodes/episode_000001 \
  --config configs/so101_gateway.yaml \
  --actuate
```

replay 동작:

```text
현재 joint state 수신
-> episode t=0 state_positions_rad로 천천히 이동
-> action_positions_rad trajectory 재생
-> 마지막 action hold
```

---

## 18. Dataset을 A6000으로 복사

데이터는 GitHub에 올리지 않는다. `rsync`, `scp`, 외장 SSD를 사용한다.

```bash
rsync -avh --progress /home/soda/Desktop/VLA/datasets/native/so101_pickplace_v1/ \
  <A6000_USER>@<A6000_HOST>:~/VLA/datasets/native/so101_pickplace_v1/
```

SSH 포트가 따로 있으면:

```bash
rsync -avh --progress -e "ssh -p <SSH_PORT>" /home/soda/Desktop/VLA/datasets/native/so101_pickplace_v1/ \
  <A6000_USER>@<A6000_HOST>:~/VLA/datasets/native/so101_pickplace_v1/
```

A6000에서 확인:

```bash
cd ~/VLA
python recording/validate_native_dataset.py \
  --root datasets/native/so101_pickplace_v1
```

---

## 19. LeRobotDataset 변환

A6000:

```bash
conda activate vla-lerobot
cd ~/VLA

python recording/convert_native_to_lerobot.py \
  --input datasets/native/so101_pickplace_v1 \
  --output datasets/lerobot/so101_pickplace_v1 \
  --repo-id local/so101_pickplace_v1
```

덮어쓰기:

```bash
python recording/convert_native_to_lerobot.py \
  --input datasets/native/so101_pickplace_v1 \
  --output datasets/lerobot/so101_pickplace_v1 \
  --repo-id local/so101_pickplace_v1 \
  --overwrite
```

변환 규칙:

| Native | LeRobot / SmolVLA |
|---|---|
| `front.mp4` | `observation.images.camera1` |
| `top.mp4` | `observation.images.camera2` |
| black image | `observation.images.camera3` |
| arm joints radian | degree |
| gripper `0.00~0.80 rad` | `0~33` |
| `/follower/joint_states` | `observation.state` |
| `/follower/joint_targets` | `action` |

---

## 20. Local Dataset으로 다시 Fine-tuning

[configs/finetune.smolvla.yaml](../configs/finetune.smolvla.yaml)을 local dataset으로 바꾼다.

```yaml
dataset:
  repo_id: null
  root: datasets/lerobot/so101_pickplace_v1
```

그 후:

```bash
conda activate vla-lerobot
cd ~/VLA

CUDA_VISIBLE_DEVICES=2 python training/run_finetune.py \
  --config configs/finetune.smolvla.yaml
```

---

## 21. A6000-only Mock 검증

실물 SO101 없이 A6000 내부에서 packet/action 흐름을 확인할 수 있다.

Terminal A:

```bash
conda activate vla-lerobot
cd ~/VLA
python policy/run_policy_server.py \
  --config configs/policy_server.smolvla.yaml
```

Terminal B:

```bash
conda activate vla-lerobot
cd ~/VLA
python remote_so101/run_robot_client.py \
  --config configs/remote_so101.yaml \
  --duration-s 60 \
  --max-steps 300 \
  --stop-after-actions 1
```

Terminal C:

```bash
conda activate vla-lerobot
cd ~/VLA
python remote_so101/mock_gateway.py \
  --config configs/mock_gateway.so101.yaml
```

---

## 22. 전체 실행 순서 요약

### 공개 dataset으로 policy 만들기

```text
A6000 validate env
-> run_finetune.py
-> checkpoint symlink
-> validate_checkpoint.py
```

### SmolVLA로 SO101 dry-run

```text
A6000 PolicyServer
-> A6000 RobotClient + bridge
-> SO101 bringup
-> SO101 SSH tunnel
-> SO101 gateway --dry-run
-> action valid/reject 로그 확인
```

### 실물 구동

```text
dry-run 정상
-> actuation_enabled=true
-> 짧은 limited workspace 실행
-> 테스트 후 actuation_enabled=false
```

### teleop dataset 만들기

```text
SO101 bringup
-> leader/joystick teleop
-> record_so101_episode.py
-> validate_native_dataset.py
-> replay_so101_episode.py dry-run
-> A6000으로 rsync
-> convert_native_to_lerobot.py
-> local dataset fine-tuning
```

---

## 23. 빠른 체크리스트

A6000:

- `conda activate vla-lerobot`
- `python training/validate_lerobot_env.py` 통과
- `checkpoints/smolvla_so101` 존재
- PolicyServer가 `0.0.0.0:8080`에서 실행
- RobotClient가 `127.0.0.1:49100` bridge를 열고 있음

SO101:

- `source ~/physicai_arm_ws/install/setup.bash`
- `source ~/vla_gateway_env/bin/activate`
- `/arm/front_cam`, `/arm/top_cam`, `/follower/joint_states` 정상
- SSH tunnel 연결
- gateway dry-run에서 valid/reject action 로그 확인
- 실물 publish 전 `actuation_enabled`와 workspace 확인

Dataset:

- native dataset validator 통과
- replay dry-run 통과
- 데이터는 GitHub가 아니라 rsync/scp/SSD로 이동
- LeRobotDataset 변환 후 local fine-tuning config 연결

---

## 24. 자주 보는 문제

### `soundfile`, protobuf, local site-package import 문제

`vla-lerobot`에서는 user site package가 섞이지 않게 한다.

```bash
PYTHONNOUSERSITE=1 python ...
```

`remote_so101/run_robot_client.py`는 user site를 제거하도록 되어 있다.

### SO101 camera topic이 안 뜸

```bash
sudo systemctl restart nvargus-daemon
```

필요하면:

```bash
bash scripts/check_so101_camera_health.sh
bash scripts/recover_so101_argus.sh
```

### recorder가 action을 못 받음

```bash
ros2 topic echo /follower/joint_targets --once
```

리더암/조이스틱 teleop node가 실제로 `/follower/joint_targets`를 publish하는지 확인한다.

### replay가 시작 자세에서 실패함

현재 로봇 자세와 episode t=0 자세 차이가 크거나, preposition 중 safety limit을 넘은 것이다.

```bash
python so101_gateway/replay_so101_episode.py \
  --episode <EPISODE_DIR> \
  --config configs/so101_gateway.yaml
```

dry-run 로그에서 `preposition_max_delta`와 frame별 max delta를 확인한다.

### 실물 gateway가 계속 reject함

대부분 다음 중 하나다.

- joint order mismatch
- joint limit 초과
- max delta 초과
- stale action
- stale sensor
- SSH tunnel disconnect

먼저 dry-run 로그를 보고, 필요하면 [SO101 Next Dry-run Guide](./so101_next_dryrun_guide.md)를 따른다.
