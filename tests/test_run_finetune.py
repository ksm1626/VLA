"""Tests for LeRobot fine-tuning command generation."""

from __future__ import annotations

import unittest

from training.run_finetune import build_command


class RunFinetuneCommandTest(unittest.TestCase):
    def _base_config(self) -> dict:
        return {
            "policy": {
                "path": "checkpoints/smolvla_so101",
                "device": "cuda",
                "push_to_hub": False,
                "empty_cameras": 1,
            },
            "dataset": {
                "repo_id": "local/so101_pickplace_v1",
                "root": "datasets/lerobot/so101_pickplace_v1",
                "rename_map": {},
            },
            "training": {
                "output_dir": "outputs/train/smolvla_so101_v2",
                "job_name": "smolvla_so101_v2_training",
                "batch_size": 64,
                "steps": 20000,
                "wandb_enable": False,
                "extra_args": [],
            },
        }

    def test_legacy_policy_path_is_supported(self) -> None:
        command = build_command(self._base_config())

        self.assertIn("--policy.path=checkpoints/smolvla_so101", command)
        self.assertIn("--policy.empty_cameras=1", command)
        self.assertNotIn("--policy.type=smolvla", command)

    def test_pi05_policy_type_and_pretrained_path_are_supported(self) -> None:
        config = self._base_config()
        config["policy"] = {
            "type": "pi05",
            "pretrained_path": "checkpoints/pi05_base",
            "device": "cuda",
            "push_to_hub": False,
            "dtype": "bfloat16",
            "gradient_checkpointing": True,
            "train_expert_only": True,
            "chunk_size": 50,
            "n_action_steps": 50,
            "use_relative_actions": False,
        }
        config["training"]["batch_size"] = 8

        command = build_command(config)

        self.assertIn("--policy.type=pi05", command)
        self.assertIn("--policy.pretrained_path=checkpoints/pi05_base", command)
        self.assertIn("--policy.dtype=bfloat16", command)
        self.assertIn("--policy.gradient_checkpointing=true", command)
        self.assertIn("--policy.use_relative_actions=false", command)

    def test_policy_selector_cannot_be_mixed(self) -> None:
        config = self._base_config()
        config["policy"]["type"] = "pi05"
        config["policy"]["pretrained_path"] = "checkpoints/pi05_base"

        with self.assertRaisesRegex(ValueError, "Use either `policy.path`"):
            build_command(config)


if __name__ == "__main__":
    unittest.main()
