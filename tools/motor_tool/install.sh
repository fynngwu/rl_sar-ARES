#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TOOL_SRC="$SCRIPT_DIR"
BUILD_DIR="$TOOL_SRC/build"

echo "=== Build motor_tool ==="

ARCH="$(uname -m)"
echo "Architecture: $ARCH"

mkdir -p "$BUILD_DIR"
cmake -S "$TOOL_SRC" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3
cmake --build "$BUILD_DIR" -j$(nproc) 2>&1 | tail -3

echo ""
echo "=== Install to ~/.local/bin ==="
mkdir -p "$HOME/.local/bin"
cp "$BUILD_DIR/motor_tool" "$HOME/.local/bin/motor_tool"
chmod +x "$HOME/.local/bin/motor_tool"

echo ""
echo "=== Done ==="
echo "  ~/.local/bin/motor_tool          # Motor & IMU debug tool"
echo ""
echo "Usage:"
echo "  motor_tool autoreport [-j N]     # Print motor pos in real-time"
echo "  motor_tool setzero [-j N]        # Set zero point"
echo "  motor_tool to_offset [-j N]      # Enable + interpolate to offset"
echo "  motor_tool imu                   # Read IMU gyro + gravity"
echo "  motor_tool errors [-j N] [-c]    # Read/clear motor errors"
