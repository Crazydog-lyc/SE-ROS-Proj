#!/usr/bin/env bash
# Alias: GUI batch (same as run_batch.sh default)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RUN_HEADLESS=False
export ENABLE_SAFETY="${ENABLE_SAFETY:-False}"
export RESULTS_DIR="${RESULTS_DIR:-/tmp/nav2_results_gui}"
exec "${SCRIPT_DIR}/run_batch.sh"
