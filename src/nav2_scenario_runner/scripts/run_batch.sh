#!/usr/bin/env bash
# Batch: generate -> run_batch -> summarize
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
PROFILE="${PROFILE:-${WS_ROOT}/src/nav2_scenario_runner/config/batch_profiles/default_batch.yaml}"
CASES_DIR="${CASES_DIR:-/tmp/nav2_cases}"
RESULTS_DIR="${RESULTS_DIR:-/tmp/nav2_results}"

profile_setting() {
  /usr/bin/python3.10 - "$PROFILE" "$1" "$2" <<'PY'
import sys
import yaml

profile_path, key, default = sys.argv[1:4]
try:
    with open(profile_path, "r", encoding="utf-8") as handle:
        profile = yaml.safe_load(handle) or {}
    value = (profile.get("batch_settings") or {}).get(key, default)
except Exception:
    value = default

if isinstance(value, bool):
    print("True" if value else "False")
else:
    print(value)
PY
}

WORLD_FILE_OVERRIDE="${WORLD_FILE:-}"
WORLD_FILE_PROFILE="$(profile_setting world_file "")"
TIMEOUT_SEC="${TIMEOUT_SEC:-$(profile_setting timeout_sec 900)}"
RUN_HEADLESS="${RUN_HEADLESS:-$(profile_setting run_headless False)}"
ENABLE_SAFETY="${ENABLE_SAFETY:-$(profile_setting enable_safety True)}"
ENABLE_SEMANTIC="${ENABLE_SEMANTIC:-$(profile_setting enable_semantic True)}"
AUTO_START_DELAY_SEC="${AUTO_START_DELAY_SEC:-$(profile_setting auto_start_delay_sec 120)}"
NAV2_STARTUP_DELAY_SEC="${NAV2_STARTUP_DELAY_SEC:-$(profile_setting nav2_startup_delay_sec 50)}"
NAV2_READY_TIMEOUT_SEC="${NAV2_READY_TIMEOUT_SEC:-$(profile_setting nav2_ready_timeout_sec 420)}"
SERVER_TIMEOUT_SEC="${SERVER_TIMEOUT_SEC:-$(profile_setting server_timeout_sec 420)}"
ROS_DOMAIN_BASE="${ROS_DOMAIN_BASE:-$(profile_setting ros_domain_base 70)}"
GENERATE_SCATTERED_WORLD="${GENERATE_SCATTERED_WORLD:-False}"
SCATTER_SEED="${SCATTER_SEED:-42}"
SCATTER_BOX_COUNT="${SCATTER_BOX_COUNT:-12}"

set +u
source /opt/ros/humble/setup.bash
source "${WS_ROOT}/install/setup.bash"
set -u

export PATH="/usr/bin:/opt/ros/humble/bin:${PATH}"
export ENABLE_SEMANTIC

if [[ "${RUN_HEADLESS}" == "True" || "${RUN_HEADLESS}" == "true" || "${RUN_HEADLESS}" == "1" ]]; then
  MODE_LABEL="headless"
else
  MODE_LABEL="GUI (Gazebo + RViz)"
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "WARNING: DISPLAY is unset; GUI mode may fail. Set RUN_HEADLESS=True for headless." >&2
  fi
fi

echo ">>> [1/3] generate_cases -> ${CASES_DIR}"
if [[ "${GENERATE_SCATTERED_WORLD}" == "True" || "${GENERATE_SCATTERED_WORLD}" == "true" || "${GENERATE_SCATTERED_WORLD}" == "1" ]]; then
  if [[ -n "${WORLD_FILE_OVERRIDE}" && "${WORLD_FILE_OVERRIDE}" != "demo_pillar_room.sdf" ]]; then
    echo "WARNING: GENERATE_SCATTERED_WORLD is enabled but WORLD_FILE=${WORLD_FILE_OVERRIDE}; generated demo_pillar_room.sdf will not be used." >&2
  elif [[ -n "${WORLD_FILE_PROFILE}" && "${WORLD_FILE_PROFILE}" != "demo_pillar_room.sdf" ]]; then
    echo "WARNING: GENERATE_SCATTERED_WORLD is enabled but batch_settings.world_file=${WORLD_FILE_PROFILE}; generated demo_pillar_room.sdf may not be used." >&2
  fi
  echo ">>> [0/3] generate_scattered_room seed=${SCATTER_SEED} boxes=${SCATTER_BOX_COUNT}"
  /usr/bin/python3.10 "${WS_ROOT}/src/sam_bot_nav2_gz/scripts/generate_scattered_room.py" \
    --seed "${SCATTER_SEED}" \
    --box-count "${SCATTER_BOX_COUNT}" \
    --output "${WS_ROOT}/src/sam_bot_nav2_gz/world/demo_pillar_room.sdf"
fi

/usr/bin/python3.10 "${SCRIPT_DIR}/generate_cases.py" \
  --profile "${PROFILE}" \
  --output-dir "${CASES_DIR}"

echo ">>> [2/3] run_batch (${MODE_LABEL}) -> ${RESULTS_DIR}"
RUN_BATCH_CMD=(
  /usr/bin/python3.10 "${SCRIPT_DIR}/run_batch.py"
  --cases-dir "${CASES_DIR}" \
  --results-dir "${RESULTS_DIR}" \
  --launch-package nav2_scenario_runner \
  --launch-file run_single_case.launch.py \
  --timeout-sec "${TIMEOUT_SEC}" \
  --ros-domain-base "${ROS_DOMAIN_BASE}" \
  --tee-output \
  --extra-launch-arg "run_headless:=${RUN_HEADLESS}" \
  --extra-launch-arg "enable_safety:=${ENABLE_SAFETY}" \
  --extra-launch-arg "spawn_from_scenario:=False" \
  --extra-launch-arg "auto_start_delay_sec:=${AUTO_START_DELAY_SEC}" \
  --extra-launch-arg "nav2_startup_delay_sec:=${NAV2_STARTUP_DELAY_SEC}" \
  --extra-launch-arg "nav2_ready_timeout_sec:=${NAV2_READY_TIMEOUT_SEC}" \
  --extra-launch-arg "server_timeout_sec:=${SERVER_TIMEOUT_SEC}"
)
if [[ -n "${WORLD_FILE_OVERRIDE}" ]]; then
  RUN_BATCH_CMD+=(--extra-launch-arg "world_file:=${WORLD_FILE_OVERRIDE}")
fi
"${RUN_BATCH_CMD[@]}"

echo ">>> [3/3] summarize"
/usr/bin/python3.10 "${SCRIPT_DIR}/summarize_results.py" \
  --results-dir "${RESULTS_DIR}"

echo "Done. See ${RESULTS_DIR}/results.csv and summary.json"
