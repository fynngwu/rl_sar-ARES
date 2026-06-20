#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BIN_DIR="$SCRIPT_DIR/src/rl_sar/build/bin"
DRIVER_LIB_DIR="$SCRIPT_DIR/driver"
ONNX_LIB_DIR="$SCRIPT_DIR/library/inference_runtime/onnxruntime/lib"
POLICY="${1:-dream_waq/dream_waq}"

source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH="$DRIVER_LIB_DIR:$ONNX_LIB_DIR:${LD_LIBRARY_PATH:-}"

trap 'kill 0; exit 0' SIGINT SIGTERM

"$BIN_DIR/ares_driver_node" "$POLICY" &
PID_DRIVER=$!

"$BIN_DIR/ares" "$POLICY" &
PID_RL=$!

wait $PID_DRIVER $PID_RL
