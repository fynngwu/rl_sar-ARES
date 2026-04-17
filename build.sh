#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

ACTION="${1:-full}"

source_ros2() {
    if [ -z "$ROS_DISTRO" ]; then
        if [ -f "/opt/ros/humble/setup.bash" ]; then
            source /opt/ros/humble/setup.bash
        else
            echo "ERROR: ROS2 not found. Source ROS2 setup.bash first."
            exit 1
        fi
    fi
}

install_bins() {
    mkdir -p "$HOME/.local/bin"
    rm -f "$HOME/.local/bin/ares" "$HOME/.local/bin/ares_driver_node"
    cp src/rl_sar/build/bin/ares src/rl_sar/build/bin/ares_driver_node "$HOME/.local/bin/"
    echo "Installed: ~/.local/bin/ares, ~/.local/bin/ares_driver_node"
}

if [ "$ACTION" = "full" ]; then
    echo "=== ARES Full Build ==="

    echo ""
    echo "[1/2] Building driver..."
    cmake -S driver -B driver/build -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3
    cmake --build driver/build -j$(nproc) 2>&1 | tail -3
    cp -u driver/build/libdog_driver.so driver/libdog_driver.so 2>/dev/null || true

    echo ""
    echo "[2/2] Building ROS2 nodes..."
    source_ros2
    cmake -S src/rl_sar -B src/rl_sar/build -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3
    cmake --build src/rl_sar/build -j$(nproc) 2>&1 | tail -3

    install_bins
    echo "=== Done ==="

elif [ "$ACTION" = "make" ]; then
    echo "=== ARES Incremental Build ==="

    cmake --build driver/build -j$(nproc) 2>&1 | tail -3
    cp -u driver/build/libdog_driver.so driver/libdog_driver.so 2>/dev/null || true

    source_ros2
    cmake --build src/rl_sar/build -j$(nproc) 2>&1 | tail -3

    install_bins
    echo "=== Done ==="

else
    echo "Usage: $0 [full|make]"
    echo "  full  - cmake configure + build (default)"
    echo "  make  - incremental build only (faster)"
    exit 1
fi
