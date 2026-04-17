#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== ARES RL Build ==="

# 1. Driver library
echo ""
echo "[1/2] Building driver (libdog_driver.so)..."
cmake -S driver -B driver/build -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3
cmake --build driver/build -j$(nproc) 2>&1 | tail -3

# Copy to driver root for easy RPATH resolution
cp -u driver/build/libdog_driver.so driver/libdog_driver.so 2>/dev/null || true

echo ""
echo "[2/2] Building ROS2 nodes..."

# Check ROS2 environment
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    else
        echo "ERROR: ROS2 not found. Source ROS2 setup.bash first."
        exit 1
    fi
fi

cmake -S src/rl_sar -B src/rl_sar/build -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3
cmake --build src/rl_sar/build -j$(nproc) 2>&1 | tail -3

echo ""
echo "=== Install to ~/.local/bin ==="
mkdir -p "$HOME/.local/bin"
rm -f "$HOME/.local/bin/ares" "$HOME/.local/bin/ares_driver_node"
cp src/rl_sar/build/bin/ares src/rl_sar/build/bin/ares_driver_node "$HOME/.local/bin/"

echo ""
echo "=== Build complete ==="
echo "  ~/.local/bin/ares              # RL inference node"
echo "  ~/.local/bin/ares_driver_node   # Hardware driver node"
