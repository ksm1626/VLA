#!/usr/bin/env bash
set -euo pipefail

ENV_NAME="${1:-vla-lerobot}"

conda create -y -n "${ENV_NAME}" python=3.12
conda run -n "${ENV_NAME}" env PYTHONNOUSERSITE=1 \
  python -m pip install 'lerobot[smolvla]' grpcio grpcio-tools

echo "Created ${ENV_NAME}. Validate with:"
echo "conda run -n ${ENV_NAME} env PYTHONNOUSERSITE=1 python training/validate_lerobot_env.py"
