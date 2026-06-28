#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BIN_DIR="$SCRIPT_DIR/src/rl_sar/build/bin"
DRIVER_LIB_DIR="$SCRIPT_DIR/driver"
ONNX_LIB_DIR="$SCRIPT_DIR/library/inference_runtime/onnxruntime/lib"
POLICY="${1:-dogv2_cts/cts}"

source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH="$DRIVER_LIB_DIR:$ONNX_LIB_DIR:${LD_LIBRARY_PATH:-}"

PID_DRIVER=""
PID_RL=""

cleanup() {
    if [[ -n "${PID_DRIVER:-}" ]] && kill -0 "$PID_DRIVER" 2>/dev/null; then
        kill "$PID_DRIVER" 2>/dev/null || true
    fi
    if [[ -n "${PID_RL:-}" ]] && kill -0 "$PID_RL" 2>/dev/null; then
        kill "$PID_RL" 2>/dev/null || true
    fi

    for _ in $(seq 1 20); do
        local driver_alive=0
        local rl_alive=0
        if [[ -n "${PID_DRIVER:-}" ]] && kill -0 "$PID_DRIVER" 2>/dev/null; then
            driver_alive=1
        fi
        if [[ -n "${PID_RL:-}" ]] && kill -0 "$PID_RL" 2>/dev/null; then
            rl_alive=1
        fi
        if [[ "$driver_alive" -eq 0 && "$rl_alive" -eq 0 ]]; then
            break
        fi
        sleep 0.1
    done

    if [[ -n "${PID_DRIVER:-}" ]] && kill -0 "$PID_DRIVER" 2>/dev/null; then
        kill -KILL "$PID_DRIVER" 2>/dev/null || true
    fi
    if [[ -n "${PID_RL:-}" ]] && kill -0 "$PID_RL" 2>/dev/null; then
        kill -KILL "$PID_RL" 2>/dev/null || true
    fi

    wait "${PID_DRIVER:-}" "${PID_RL:-}" 2>/dev/null || true
}

trap 'cleanup; exit 0' SIGINT SIGTERM

"$BIN_DIR/ares_driver_node" "$POLICY" &
PID_DRIVER=$!

"$BIN_DIR/ares" "$POLICY" &
PID_RL=$!

while true; do
    driver_alive=0
    rl_alive=0

    if kill -0 "$PID_DRIVER" 2>/dev/null; then
        driver_alive=1
    fi
    if kill -0 "$PID_RL" 2>/dev/null; then
        rl_alive=1
    fi

    if [[ "$driver_alive" -eq 0 || "$rl_alive" -eq 0 ]]; then
        break
    fi

    sleep 0.2
done

cleanup
