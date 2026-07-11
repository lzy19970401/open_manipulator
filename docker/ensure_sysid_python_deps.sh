#!/bin/bash
# Restore numpy/scipy ABI compatibility after pip installs (e.g. figaroh) upgrade
# numpy to 2.x in /usr/local while apt scipy 1.11 remains on numpy 1.x.
set -euo pipefail

pip3 install 'numpy<2' --break-system-packages --force-reinstall

python3 - <<'PY'
import numpy
from scipy.linalg import qr

print(f'scipy.linalg.qr OK (numpy {numpy.__version__})')
PY
