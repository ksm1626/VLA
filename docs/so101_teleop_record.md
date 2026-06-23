# SO101 Teleop Dataset Recording Guide

이 문서는 SO101 PC에서 teleop 데이터를 로컬 native dataset으로 수집하고, 나중에 A6000에서 LeRobotDataset으로 변환하는 절차다.

핵심 원칙:

- SO101에는 LeRobot, torch, transformers를 설치하지 않는다.
- SO101에서는 제조사 ROS2 topic을 구독해서 `MP4 + JSONL + metadata`로 저장한다.
- A6000에서 native dataset을 LeRobotDataset으로 변환한다.
- 데이터 파일은 GitHub에 올리지 않는다. `rsync`, `scp`, 외장 SSD를 사용한다.

전체 흐름:

```text
SO101 제조사 ROS2 bringup
  /arm/front_cam
  /arm/top_cam
  /follower/joint_states
  /follower/joint_targets
        |
        v
record_so101_episode.py
        |
        v
~/so101_datasets/so101_pickplace_v1/
  episodes/
    episode_000001/
      frames.jsonl
      videos/front.mp4
      videos/top.mp4
      episode.json
    episode_000002/
      frames.jsonl
      videos/front.mp4
      videos/top.mp4
      episode.json
    ...
        |
        v
A6000으로 복사
        |
        v
convert_native_to_lerobot.py
        |
        v
datasets/lerobot/so101_pickplace_v1/
```

---

## 1. State와 Action 기준

recorder가 학습용으로 쓰는 핵심 topic은 두 개다.

| 의미 | ROS topic | 설명 |
|---|---|---|
| state | `/follower/joint_states` | 현재 로봇 관절 위치 |
| action | `/follower/joint_targets` | 로봇에게 보낸 목표 관절 위치 |

둘 다 6D joint vector지만 의미가 다르다.

```text
state  = 지금 로봇이 실제로 어디에 있는가
action = 지금 로봇에게 어디로 가라고 명령했는가
```

리더암을 쓰든 조이스틱을 쓰든 학습 action은 항상 `/follower/joint_targets`다.

```text
leader arm -> /leader/joint_states -> teleoperation -> /follower/joint_targets
joystick   -> /joy -> ik_calc       -> /follower/joint_targets
```

`/leader/joint_states`, `/joy`, `/target_pose`, `/gripper_open`은 debug metadata로만 저장한다.

---

## 2. SO101 코드 준비

SO101 PC에서 repo를 최신으로 맞춘다.

```bash
cd ~/Desktop/VLA
git pull
```

만약 다른 위치에 clone했다면 해당 위치로 이동한다.

```bash
cd ~/VLA
git pull
```

SO101 Python 환경은 기존 gateway 환경을 그대로 쓴다.

```bash
source ~/physicai_arm_ws/install/setup.bash
source ~/vla_gateway_env/bin/activate
cd ~/Desktop/VLA
```

의존성 확인:

```bash
python - <<'PY'
import rclpy
import cv2
import numpy
import yaml

print("SO101 recorder deps OK")
print("opencv", cv2.__version__)
PY
```

정상 예:

```text
SO101 recorder deps OK
opencv 4.10.0
```

주의:

- `numpy`, `opencv`, `torch`를 업그레이드하지 않는다.
- SO101에는 LeRobot을 설치하지 않는다.

---

## 3. 제조사 Bringup 실행

터미널 1:

```bash
source ~/physicai_arm_ws/install/setup.bash
ros2 launch physicai_arm bringup.launch.py
```

정상적으로 필요한 topic이 떠야 한다.

터미널 2:

```bash
source ~/physicai_arm_ws/install/setup.bash

ros2 topic list | grep -E "arm|follower|leader|joy|target_pose|gripper"
```

필수 topic:

```text
/arm/front_cam
/arm/top_cam
/follower/joint_states
/follower/joint_targets
```

카메라 hz 확인:

```bash
ros2 topic hz /arm/front_cam
```

다른 터미널:

```bash
ros2 topic hz /arm/top_cam
```

정상 기준:

```text
average rate: ...
```

카메라 topic이 안 뜨거나 멈추면 먼저 카메라를 복구한다.

```bash
sudo systemctl restart nvargus-daemon
```

그 후 bringup을 다시 실행한다.

---

## 4. Teleop 방식 선택

### Option A. 리더암 Teleop

리더암 USB가 연결되어 있어야 한다.

```bash
ls /dev/ttyACM*
```

기본 기대값:

```text
/dev/ttyACM0
/dev/ttyACM1
```

제조사 config 기준:

```text
/dev/ttyACM0 = follower
/dev/ttyACM1 = leader
```

bringup 안에서 leader driver가 정상 실행되면 `/leader/joint_states`가 나온다.

```bash
source ~/physicai_arm_ws/install/setup.bash
ros2 topic echo /leader/joint_states --once
```

리더암 값을 follower target으로 보내는 teleoperation 실행:

```bash
source ~/physicai_arm_ws/install/setup.bash
ros2 run physicai_arm teleoperation --ros-args \
  -p input_topic:=/leader/joint_states \
  -p output_topic:=/follower/joint_targets
```

주의:

- 처음 실행할 때 leader와 follower 자세 차이가 크면 follower가 갑자기 움직일 수 있다.
- follower 주변을 비우고 전원/정지 수단을 바로 잡을 수 있는 상태에서 시작한다.

### Option B. 조이스틱 Teleop

조이스틱 연결 확인:

```bash
ls /dev/input/js0
```

터미널 2-1:

```bash
source ~/physicai_arm_ws/install/setup.bash
ros2 run joy joy_node --ros-args -p device:=/dev/input/js0
```

터미널 2-2:

```bash
source ~/physicai_arm_ws/install/setup.bash
ros2 run physicai_arm joy_to_target
```

터미널 2-3:

```bash
source ~/physicai_arm_ws/install/setup.bash
ros2 run physicai_arm ik_calc --ros-args \
  -p joint_topic:=/follower/joint_states \
  -p output_topic:=/follower/joint_targets
```

정상 확인:

```bash
ros2 topic echo /follower/joint_targets --once
```

이 topic이 나와야 recorder가 action을 기록할 수 있다.

---

## 5. Episode 기록 시작

터미널 3:

```bash
source ~/physicai_arm_ws/install/setup.bash
source ~/vla_gateway_env/bin/activate
cd ~/Desktop/VLA

python so101_gateway/record_so101_episode.py \
  --config configs/so101_recording.yaml \
  --task "Pick up the cube and place it in the target bin" \
  --teleop-source leader
```

조이스틱 데이터면 `--teleop-source joystick`으로 바꾼다.

```bash
python so101_gateway/record_so101_episode.py \
  --config configs/so101_recording.yaml \
  --task "Pick up the cube and place it in the target bin" \
  --teleop-source joystick
```

정상 시작 예:

```text
Recording episode_000001 at 10 FPS
Task: Pick up the cube and place it in the target bin
Teleop source: leader
Controls: Enter=save+next, c=discard+next, q=save+quit, Ctrl+C=save+quit
```

아직 필수 topic이 준비되지 않았으면 다음처럼 대기한다.

```text
Waiting for recording inputs: joint target action not received yet
```

이 경우 teleop이 `/follower/joint_targets`를 만들고 있는지 확인한다.

기록 중 조작:

| 입력 | 동작 |
|---|---|
| `Enter` | 현재 episode 저장 후 다음 episode 바로 시작 |
| `c` + `Enter` | 현재 episode 버리고 다음 episode 바로 시작 |
| `q` + `Enter` | 현재 episode 저장 후 종료 |
| `Ctrl+C` | 현재 episode 저장 후 종료 |

예를 들어 첫 번째 episode를 끝내고 바로 두 번째 episode를 찍고 싶으면 recorder 터미널에서 그냥 `Enter`를 누른다.

정상 저장 후 다음 episode 시작 예:

```text
Saved /home/soda/so101_datasets/so101_pickplace_v1/episodes/episode_000001
Frames: 342
Recording episode_000002 at 10 FPS
```

현재 episode가 마음에 들지 않으면 `c`를 입력하고 `Enter`를 누른다.

```text
Discarded episode_000002: user discarded
Recording episode_000002 at 10 FPS
```

종료하려면 `q` 입력 후 `Enter`, 또는 `Ctrl+C`를 사용한다.

```text
Saved /home/soda/so101_datasets/so101_pickplace_v1/episodes/episode_000002
Frames: 318
```

`recording.min_frames_per_episode`보다 frame 수가 적은 episode는 저장하지 않고 자동 discard한다. 기본값은 1 frame이다.

---

## 6. 저장 결과 구조

기본 저장 위치는 [configs/so101_recording.yaml](../configs/so101_recording.yaml)의 `dataset.root`다.

```yaml
dataset:
  root: ~/so101_datasets/so101_pickplace_v1
```

저장 결과:

```text
~/so101_datasets/so101_pickplace_v1/
  dataset.yaml
  episodes/
    episode_000001/
      episode.json
      frames.jsonl
      videos/
        front.mp4
        top.mp4
    episode_000002/
      episode.json
      frames.jsonl
      videos/
        front.mp4
        top.mp4
```

`frames.jsonl`에는 frame마다 다음 정보가 들어간다.

```text
frame_index
timestamp_ns
relative_time_s
task
teleop_source
joint_names
state_positions_rad
action_positions_rad
debug.leader_positions_rad
debug.joy_axes
debug.target_pose_position
debug.gripper_open
```

원본 native dataset은 radian 기준이다.

```text
state_positions_rad  = /follower/joint_states
action_positions_rad = /follower/joint_targets
action_type          = absolute_joint_position
```

---

## 7. SO101에서 Native Dataset 검증

SO101에서 바로 검증:

```bash
source ~/vla_gateway_env/bin/activate
cd ~/Desktop/VLA

python recording/validate_native_dataset.py \
  --root ~/so101_datasets/so101_pickplace_v1
```

정상 예:

```text
VALID native dataset: /home/soda/so101_datasets/so101_pickplace_v1
Episodes: 2
Frames: 660
- episode_000001: frames=342 teleop_source=leader task=Pick up the cube and place it in the target bin
- episode_000002: frames=318 teleop_source=leader task=Pick up the cube and place it in the target bin
```

검증기가 확인하는 것:

- `dataset.yaml` 존재
- `episode.json` 존재
- `frames.jsonl` 존재
- `videos/front.mp4`, `videos/top.mp4` 존재
- JSONL row 수와 MP4 frame 수 일치
- state/action 차원 6
- joint order 일치
- NaN/Inf 없음

---

## 8. SO101에서 A6000으로 데이터 복사

GitHub에는 데이터 파일을 올리지 않는다.

SO101에서 A6000으로 복사:

```bash
rsync -avh --progress ~/so101_datasets/so101_pickplace_v1/ \
  pnudtn10@A6000_IP:~/VLA/datasets/native/so101_pickplace_v1/
```

예를 들어 SSH 포트가 따로 있으면:

```bash
rsync -avh --progress -e "ssh -p 6519" ~/so101_datasets/so101_pickplace_v1/ \
  pnudtn10@164.125.19.141:~/VLA/datasets/native/so101_pickplace_v1/
```

A6000에서 확인:

```bash
cd ~/VLA
python recording/validate_native_dataset.py \
  --root datasets/native/so101_pickplace_v1
```

---

## 9. A6000에서 LeRobotDataset으로 변환

A6000 터미널:

```bash
conda activate vla-lerobot
cd ~/VLA

python recording/convert_native_to_lerobot.py \
  --input datasets/native/so101_pickplace_v1 \
  --output datasets/lerobot/so101_pickplace_v1 \
  --repo-id local/so101_pickplace_v1
```

이미 output이 있으면 덮어쓰기:

```bash
python recording/convert_native_to_lerobot.py \
  --input datasets/native/so101_pickplace_v1 \
  --output datasets/lerobot/so101_pickplace_v1 \
  --repo-id local/so101_pickplace_v1 \
  --overwrite
```

변환 규칙:

| native | LeRobot / SmolVLA |
|---|---|
| `front.mp4` | `observation.images.camera1` |
| `top.mp4` | `observation.images.camera2` |
| black image | `observation.images.camera3` |
| arm joints radian | degree |
| gripper `0.00~0.80 rad` | `0~33` |
| `/follower/joint_states` | `observation.state` |
| `/follower/joint_targets` | `action` |

변환 결과:

```text
datasets/lerobot/so101_pickplace_v1/
  meta/
  data/
  videos/
```

---

## 10. Fine-tuning Config에 연결

[configs/finetune.smolvla.yaml](../configs/finetune.smolvla.yaml)에서 local dataset을 쓰려면 `dataset.root`를 변환 결과로 지정한다.

예:

```yaml
dataset:
  repo_id: null
  root: datasets/lerobot/so101_pickplace_v1
```

그 후 A6000에서 학습:

```bash
conda activate vla-lerobot
cd ~/VLA

CUDA_VISIBLE_DEVICES=2 python training/run_finetune.py \
  --config configs/finetune.smolvla.yaml
```

---

## 11. 자주 나는 문제

### `/follower/joint_targets`를 못 받아서 recorder가 대기함

증상:

```text
Waiting for recording inputs: joint target action not received yet
```

확인:

```bash
ros2 topic echo /follower/joint_targets --once
```

리더암이면 `teleoperation` 노드가 실행 중인지 확인한다. 조이스틱이면 `joy_node`, `joy_to_target`, `ik_calc` 세 노드가 모두 실행 중인지 확인한다.

### 카메라 frame을 못 받음

확인:

```bash
ros2 topic hz /arm/front_cam
ros2 topic hz /arm/top_cam
```

필요하면:

```bash
sudo systemctl restart nvargus-daemon
```

그 후 bringup을 다시 실행한다.

### validator에서 video frame 수가 안 맞음

녹화 중 프로세스가 강제 종료됐거나 MP4 finalize가 실패했을 가능성이 있다. 정상 종료는 `q` + `Enter` 또는 `Ctrl+C` 한 번으로 recorder가 저장 메시지를 출력할 때까지 기다린다.

### LeRobot 변환 중 vcodec 에러

기본값은 CPU `h264`다. A6000에서 GPU 인코딩을 쓰고 싶으면 직접 지정할 수 있다.

```bash
python recording/convert_native_to_lerobot.py \
  --input datasets/native/so101_pickplace_v1 \
  --output datasets/lerobot/so101_pickplace_v1 \
  --repo-id local/so101_pickplace_v1 \
  --vcodec h264_nvenc
```

인코더 문제가 있으면 기본 `h264`로 다시 실행한다.

---

## 12. 최소 체크리스트

SO101에서 episode를 찍기 전:

- `/arm/front_cam` hz가 나온다.
- `/arm/top_cam` hz가 나온다.
- `/follower/joint_states`가 나온다.
- teleop 실행 후 `/follower/joint_targets`가 나온다.
- recorder가 `Recording episode_...`를 출력한다.

episode 종료 후:

- `Enter`, `q` + `Enter`, 또는 `Ctrl+C` 후 `Saved ...`가 출력된다.
- 실패한 episode는 `c` + `Enter`로 버릴 수 있다.
- `python recording/validate_native_dataset.py --root ~/so101_datasets/so101_pickplace_v1`가 통과한다.

A6000에서:

- native dataset을 `datasets/native/so101_pickplace_v1/`로 복사했다.
- validator가 통과한다.
- `convert_native_to_lerobot.py`가 통과한다.
- `datasets/lerobot/so101_pickplace_v1/`가 생성된다.
