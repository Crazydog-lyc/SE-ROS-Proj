#!/usr/bin/env bash
# 一键质量门禁：自动 source ROS + 工作空间后跑 pytest/覆盖率
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="$(cd "${PKG_ROOT}/../.." && pwd)"

# ROS setup.bash 在 set -u 下可能报未绑定变量，先关闭 nounset
set +u
source /opt/ros/humble/setup.bash
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  source "${WS_ROOT}/install/setup.bash"
fi
set -u

cd "${PKG_ROOT}"
export PYTHONPATH="${PKG_ROOT}:${PYTHONPATH:-}"
exec python3.10 "${SCRIPT_DIR}/run_quality_gate.py" "$@"
