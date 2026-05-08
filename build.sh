#!/bin/bash
set -e
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

ACTION="${1:-make}"

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
    cp src/rl_sar/build/bin/ares src/rl_sar/build/bin/ares_driver_node "$HOME/.local/bin/"
    echo "Installed: ~/.local/bin/ares, ~/.local/bin/ares_driver_node"
}

if [ "$ACTION" = "full" ]; then
    echo "=== ARES Full Build ==="

    echo ""
    echo "[1/2] Building driver..."
    cmake -S driver -B driver/build -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3
    cmake --build driver/build --target dog_driver -j$(nproc) 2>&1 | tail -3
    cp -u driver/build/libdog_driver.so driver/libdog_driver.so 2>/dev/null || true

    echo ""
    echo "[2/2] Building ROS2 nodes..."
    source_ros2
    cmake -S src/rl_sar -B src/rl_sar/build -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3
    cmake --build src/rl_sar/build --target ares ares_driver_node -j$(nproc) 2>&1 | tail -3

    install_bins
    echo "=== Done ==="

elif [ "$ACTION" = "make" ]; then
    echo "=== ARES Incremental Build ==="

    if [ ! -f driver/build/Makefile ]; then
        cmake -S driver -B driver/build -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3
    fi
    cmake --build driver/build --target dog_driver -j$(nproc) 2>&1 | tail -3
    cp -u driver/build/libdog_driver.so driver/libdog_driver.so 2>/dev/null || true

    source_ros2
    if [ ! -f src/rl_sar/build/Makefile ]; then
        cmake -S src/rl_sar -B src/rl_sar/build -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3
    fi
    cmake --build src/rl_sar/build --target ares ares_driver_node -j$(nproc) 2>&1 | tail -3

    install_bins
    echo "=== Done ==="

else
    echo "Usage: $0 [full|make]"
    echo "  make  - incremental build; configures once if needed (default)"
    echo "  full  - cmake configure + build"
    exit 1
fi
