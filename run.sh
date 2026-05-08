#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BIN_DIR="$SCRIPT_DIR/build/rl_sar/bin"
POLICY="${1:-ares_himloco/himloco}"

trap 'kill 0; exit 0' SIGINT SIGTERM

"$BIN_DIR/ares_driver_node" "$POLICY" &
PID_DRIVER=$!

"$BIN_DIR/ares" "$POLICY" &
PID_RL=$!

wait $PID_DRIVER $PID_RL
