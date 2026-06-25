#!/usr/bin/env python3
"""Validate a fine-tuned LeRobot policy checkpoint without ROS2 or robot hardware."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import torch

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "training"))

from lerobot_config import load_yaml_config, require_mapping, require_value  # noqa: E402


def _resolve_checkpoint_from_config(config_path: str | Path) -> Path:
    config = load_yaml_config(config_path)
    defaults = require_mapping(config, "client_handshake_defaults")
    return Path(require_value(defaults, "pretrained_name_or_path", "client_handshake_defaults"))


def _load_train_config(checkpoint: Path) -> dict:
    train_config_path = checkpoint / "train_config.json"
    if not train_config_path.exists():
        raise FileNotFoundError(f"Missing train_config.json in checkpoint: {checkpoint}")
    with train_config_path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _has_train_config(checkpoint: Path) -> bool:
    return (checkpoint / "train_config.json").exists()


def _sample_to_batch(sample: dict) -> dict:
    batch = {}
    for key, value in sample.items():
        if torch.is_tensor(value):
            batch[key] = value.unsqueeze(0)
        else:
            batch[key] = [value]
    return batch


def _validate_policy_on_dataset(
    *,
    checkpoint: Path,
    policy_cfg,
    dataset,
    rename_map: dict,
    device: str,
    sample_index: int,
    actions_per_chunk: int,
) -> None:
    """Run one dataset sample through a LeRobot policy config."""
    from lerobot.policies.factory import make_policy, make_pre_post_processors

    policy_cfg.device = device

    start = time.perf_counter()
    policy = make_policy(policy_cfg, ds_meta=dataset.meta, rename_map=rename_map)
    load_s = time.perf_counter() - start

    processor_kwargs = {
        "dataset_stats": dataset.meta.stats,
        "preprocessor_overrides": {
            "device_processor": {"device": device},
            "normalizer_processor": {
                "stats": dataset.meta.stats,
                "features": {**policy.config.input_features, **policy.config.output_features},
                "norm_map": policy.config.normalization_mapping,
            },
            "rename_observations_processor": {"rename_map": rename_map},
        },
    }
    postprocessor_kwargs = {
        "postprocessor_overrides": {
            "unnormalizer_processor": {
                "stats": dataset.meta.stats,
                "features": policy.config.output_features,
                "norm_map": policy.config.normalization_mapping,
            },
        }
    }
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy_cfg,
        pretrained_path=policy_cfg.pretrained_path,
        **processor_kwargs,
        **postprocessor_kwargs,
    )

    sample = dataset[sample_index]
    batch = preprocessor(_sample_to_batch(sample))

    start = time.perf_counter()
    with torch.no_grad():
        action_chunk = policy.predict_action_chunk(batch)
        if action_chunk.ndim != 3:
            action_chunk = action_chunk.unsqueeze(0)
        action_chunk = action_chunk[:, :actions_per_chunk, :]
        postprocessed_actions = [
            postprocessor(action_chunk[:, timestep, :]) for timestep in range(action_chunk.shape[1])
        ]
        action_chunk_post = torch.stack(postprocessed_actions, dim=1).squeeze(0)
    inference_s = time.perf_counter() - start

    raw_finite = bool(torch.isfinite(action_chunk).all())
    post_finite = bool(torch.isfinite(action_chunk_post).all())
    if not raw_finite or not post_finite:
        raise ValueError(f"Non-finite action detected: raw={raw_finite}, postprocessed={post_finite}")

    print(f"policy_type={policy.config.type}")
    print(f"policy_device={device}")
    print(f"dataset_frames={len(dataset)}")
    print(f"input_features={sorted(policy.config.input_features)}")
    print(f"output_features={policy.config.output_features}")
    print(f"normalization_mapping={policy.config.normalization_mapping}")
    print(f"load_seconds={load_s:.3f}")
    print(f"inference_seconds={inference_s:.3f}")
    print(f"raw_action_chunk_shape={tuple(action_chunk.shape)} finite={raw_finite}")
    print(f"postprocessed_action_chunk_shape={tuple(action_chunk_post.shape)} finite={post_finite}")
    print(f"first_postprocessed_action={action_chunk_post[0].detach().cpu().tolist()}")
    print("checkpoint validation passed.")


def _validate_finetuned_checkpoint(
    *,
    checkpoint: Path,
    device: str,
    sample_index: int,
    actions_per_chunk: int,
) -> None:
    """Validate a checkpoint that includes LeRobot train_config.json."""
    from lerobot.configs.train import TrainPipelineConfig
    from lerobot.datasets.factory import make_dataset

    train_config = _load_train_config(checkpoint)
    print(f"checkpoint={checkpoint}")
    print(f"dataset.repo_id={train_config.get('dataset', {}).get('repo_id')}")
    print(f"dataset.root={train_config.get('dataset', {}).get('root')}")
    print(f"rename_map={train_config.get('rename_map')}")

    cfg = TrainPipelineConfig.from_pretrained(checkpoint)
    _validate_policy_on_dataset(
        checkpoint=checkpoint,
        policy_cfg=cfg.policy,
        dataset=make_dataset(cfg),
        rename_map=cfg.rename_map,
        device=device,
        sample_index=sample_index,
        actions_per_chunk=actions_per_chunk,
    )


def _validate_base_checkpoint(
    *,
    checkpoint: Path,
    policy_type: str,
    dataset_repo_id: str,
    dataset_root: Path | None,
    device: str,
    sample_index: int,
    actions_per_chunk: int,
) -> None:
    """Validate a base policy checkpoint with explicit dataset metadata."""
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.factory import make_policy_config

    print(f"checkpoint={checkpoint}")
    print(f"dataset.repo_id={dataset_repo_id}")
    print(f"dataset.root={dataset_root}")
    print("rename_map={}")

    try:
        policy_cfg = PreTrainedConfig.from_pretrained(checkpoint)
    except Exception:
        policy_cfg = make_policy_config(policy_type)

    if policy_cfg.type != policy_type:
        raise ValueError(f"Checkpoint policy type is {policy_cfg.type!r}, expected {policy_type!r}")
    policy_cfg.pretrained_path = checkpoint

    dataset = LeRobotDataset(dataset_repo_id, root=dataset_root)
    for key in ("observation.state", "action"):
        stats = dataset.meta.stats.get(key, {})
        quantile_keys = {"q01", "q99"}.intersection(stats)
        print(f"{key}.quantile_stats_present={bool(quantile_keys)}")

    _validate_policy_on_dataset(
        checkpoint=checkpoint,
        policy_cfg=policy_cfg,
        dataset=dataset,
        rename_map={},
        device=device,
        sample_index=sample_index,
        actions_per_chunk=actions_per_chunk,
    )


def validate_checkpoint(
    checkpoint: Path,
    device: str,
    sample_index: int,
    actions_per_chunk: int,
    policy_type: str | None = None,
    dataset_repo_id: str | None = None,
    dataset_root: Path | None = None,
) -> None:
    """Load a checkpoint, run one dataset sample through it, and print key checks."""
    if not checkpoint.exists():
        raise FileNotFoundError(f"Checkpoint path does not exist: {checkpoint}")

    if _has_train_config(checkpoint):
        _validate_finetuned_checkpoint(
            checkpoint=checkpoint,
            device=device,
            sample_index=sample_index,
            actions_per_chunk=actions_per_chunk,
        )
        return

    if not policy_type or not dataset_repo_id:
        raise ValueError(
            "Base checkpoint validation requires `--policy-type` and `--dataset-repo-id` "
            "when train_config.json is not present"
        )
    _validate_base_checkpoint(
        checkpoint=checkpoint,
        policy_type=policy_type,
        dataset_repo_id=dataset_repo_id,
        dataset_root=dataset_root,
        device=device,
        sample_index=sample_index,
        actions_per_chunk=actions_per_chunk,
    )


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=None,
        help="Checkpoint directory containing config.json and model.safetensors.",
    )
    parser.add_argument(
        "--config",
        default="configs/policy_server.smolvla.yaml",
        help="Policy server config used when --checkpoint is omitted.",
    )
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--sample-index", type=int, default=0)
    parser.add_argument("--actions-per-chunk", type=int, default=5)
    parser.add_argument("--policy-type", default=None)
    parser.add_argument("--dataset-repo-id", default=None)
    parser.add_argument("--dataset-root", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    """Validate the configured checkpoint."""
    args = parse_args()
    checkpoint = args.checkpoint or _resolve_checkpoint_from_config(args.config)
    validate_checkpoint(
        checkpoint=checkpoint,
        device=args.device,
        sample_index=args.sample_index,
        actions_per_chunk=args.actions_per_chunk,
        policy_type=args.policy_type,
        dataset_repo_id=args.dataset_repo_id,
        dataset_root=args.dataset_root,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
