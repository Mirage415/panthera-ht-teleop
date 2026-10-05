#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
command -v uv >/dev/null || { echo 'Install uv first (https://docs.astral.sh/uv/getting-started/installation/).' >&2; exit 1; }
git submodule update --init --recursive
uv venv --python python3.10 .venv
uv pip install --python .venv/bin/python -r requirements.lock.txt
# Upstream filename says 1.2.0; wheel metadata says 1.0.0.
UV_SKIP_WHEEL_FILENAME_CHECK=1 uv pip install --python .venv/bin/python \
  Panthera-HT_SDK/panthera_python/motor_whl/hightorque_robot-1.2.0-cp310-cp310-linux_x86_64.whl
if [[ ! -f teleop-config.json ]]; then
  cp teleop-config.example.json teleop-config.json
fi
.venv/bin/python -c 'import hightorque_robot, pinocchio, numpy, scipy; print("SDK imports OK")'
echo 'Set verified leader/follower USB paths in teleop-config.json before hardware use.'
