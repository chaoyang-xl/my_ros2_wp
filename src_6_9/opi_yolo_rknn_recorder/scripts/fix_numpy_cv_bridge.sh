#!/usr/bin/env bash

set -Eeuo pipefail

echo "[INFO] Current Python:"
which python3
python3 --version

echo "[INFO] Current numpy, if importable:"
python3 - << 'PYEOF' || true
try:
    import numpy
    print("numpy version:", numpy.__version__)
    print("numpy file:", numpy.__file__)
except Exception as e:
    print("numpy import failed:", repr(e))
PYEOF

echo "[INFO] Downgrading numpy to 1.26.4 for ROS Humble cv_bridge..."

python3 -m pip uninstall -y numpy || true

python3 -m pip install --user --force-reinstall --no-cache-dir \
  "numpy==1.26.4" \
  -i https://mirrors.aliyun.com/pypi/simple/ \
  --timeout 300 \
  --retries 10 \
  --prefer-binary

echo "[INFO] Verifying numpy:"
python3 - << 'PYEOF'
import numpy
print("numpy version:", numpy.__version__)
print("numpy file:", numpy.__file__)
assert numpy.__version__.startswith("1."), "numpy is still not 1.x"
PYEOF

echo "[INFO] Verifying cv_bridge:"
set +u
source /opt/ros/humble/setup.bash
set -u

python3 - << 'PYEOF'
from cv_bridge import CvBridge
print("cv_bridge OK")
PYEOF

echo "[INFO] Done. NumPy/cv_bridge compatibility fixed."
