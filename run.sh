#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BIN_DIR="$SCRIPT_DIR/build/rl_sar/bin"

trap 'kill 0; exit 0' SIGINT SIGTERM

"$BIN_DIR/ares_driver_node" &
PID_DRIVER=$!

"$BIN_DIR/ares" &
PID_RL=$!

wait $PID_DRIVER $PID_RL
