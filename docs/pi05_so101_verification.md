# Pi0.5 SO101 Thin Verification Guide

이 문서는 Pi0.5를 SO101 프로젝트에 붙인 뒤, 긴 fine-tuning 전에 최소한으로 확인할 절차만 정리합니다.
SO101 실물 로봇은 fine-tuned checkpoint 검증이 끝난 뒤에만 연결합니다.

## 1. Base Checkpoint 배치

Pi0.5 base model은 A6000에 아래 경로로 둡니다.

```bash
cd ~/VLA
ls checkpoints/pi05_base
```

정상 기준:

```text
config.json
model...
```

파일명은 checkpoint 형식에 따라 다를 수 있지만, LeRobot `from_pretrained`가 읽을 수 있는 디렉토리여야 합니다.

## 2. Fine-tuning Command Dry-run

```bash
conda activate vla-lerobot
cd ~/VLA

CUDA_VISIBLE_DEVICES=6 python training/run_finetune.py \
  --config configs/finetune.pi05_so101_v1.yaml \
  --dry-run
```

정상 기준:

```text
--policy.type=pi05
--policy.pretrained_path=checkpoints/pi05_base
--dataset.root=datasets/lerobot/so101_pickplace_v1
--policy.chunk_size=50
--policy.n_action_steps=50
```

## 3. Base Pi0.5 Offline Inference

실물 제어 전에 dataset sample 하나로 action chunk가 나오는지만 확인합니다.

```bash
CUDA_VISIBLE_DEVICES=6 python policy/validate_checkpoint.py \
  --policy-type pi05 \
  --checkpoint checkpoints/pi05_base \
  --dataset-repo-id local/so101_pickplace_v1 \
  --dataset-root datasets/lerobot/so101_pickplace_v1 \
  --device cuda \
  --actions-per-chunk 5
```

정상 기준:

```text
policy_type=pi05
dataset_frames=40635
observation.state.quantile_stats_present=True
action.quantile_stats_present=True
raw_action_chunk_shape=(1, 5, 6)
postprocessed_action_chunk_shape=(5, 6)
checkpoint validation passed.
```

이 단계는 base 모델로 실물 로봇을 움직이는 단계가 아닙니다.

## 4. Pi0.5 Fine-tuning 실행

```bash
CUDA_VISIBLE_DEVICES=6 python training/run_finetune.py \
  --config configs/finetune.pi05_so101_v1.yaml
```

정상 기준:

```text
policy_type=pi05
dataset.num_frames=40635
dataset.num_episodes=70
Output dir: outputs/train/pi05_so101_v1
Training: ...
```

메모리가 부족하면 `configs/finetune.pi05_so101_v1.yaml`에서 `training.batch_size`를 `4`로 낮춥니다.

## 5. Fine-tuned Checkpoint 검증

학습이 5000 step까지 끝났다면:

```bash
CUDA_VISIBLE_DEVICES=6 python policy/validate_checkpoint.py \
  --checkpoint outputs/train/pi05_so101_v1/checkpoints/005000/pretrained_model \
  --device cuda \
  --actions-per-chunk 5
```

정상 기준:

```text
policy_type=pi05
postprocessed_action_chunk_shape=(5, 6)
finite=True
checkpoint validation passed.
```

## 6. RobotClient Dry-run

PolicyServer는 기존처럼 host/port만 엽니다. 실제 policy 정보는 RobotClient handshake에서 넘어갑니다.

```bash
python policy/run_policy_server.py \
  --config configs/policy_server.pi05.yaml \
  --dry-run

python remote_so101/run_robot_client.py \
  --config configs/remote_so101.pi05.yaml \
  --dry-run
```

RobotClient dry-run 정상 기준:

```text
policy_server=127.0.0.1:8080
policy_type=pi05
checkpoint=outputs/train/pi05_so101_v1/checkpoints/005000/pretrained_model
actions_per_chunk=50
```

## 7. Normalization Fallback

Pi0.5 기본값은 state/action에 `QUANTILES` normalization을 씁니다. 현재 dataset에는 `q01/q99` 통계가 있으므로 우선 기본값으로 시도합니다.

만약 normalization 관련 에러가 나면 `configs/finetune.pi05_so101_v1.yaml`의 `training.extra_args`에 아래를 추가해 `MEAN_STD`로 우회합니다.

```yaml
training:
  extra_args:
    - --policy.normalization_mapping={"ACTION":"MEAN_STD","STATE":"MEAN_STD","VISUAL":"IDENTITY"}
```
