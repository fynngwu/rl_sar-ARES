#!/bin/bash
set -e
set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

ACTION="${1:-make}"
ICEORYX_VERSION="v2.95.4"
ICEORYX_DIR="$SCRIPT_DIR/.deps/iceoryx"
ICEORYX_INSTALL_DIR="$SCRIPT_DIR/.deps/iceoryx_install"

ensure_iceoryx() {
    if cmake --find-package -DNAME=iceoryx_posh -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST >/dev/null 2>&1; then
        echo "[iceoryx] Found system-installed iceoryx_posh"
        return 0
    fi

    if [ -f "$ICEORYX_INSTALL_DIR/lib/cmake/iceoryx_posh/iceoryx_poshConfig.cmake" ] || \
       [ -f "$ICEORYX_INSTALL_DIR/lib64/cmake/iceoryx_posh/iceoryx_poshConfig.cmake" ]; then
        local installed_ver=""
        if [ -f "$ICEORYX_DIR/VERSION_NUMBER" ]; then
            installed_ver=$(cat "$ICEORYX_DIR/VERSION_NUMBER" 2>/dev/null | tr -d '[:space:]')
        fi
        if [ "$installed_ver" = "2.95.4" ]; then
            echo "[iceoryx] Found local iceoryx v2.95.4 at $ICEORYX_INSTALL_DIR"
            return 0
        fi
        echo "[iceoryx] Local iceoryx version mismatch ($installed_ver != 2.95.4), reinstalling..."
    fi

    install_iceoryx
}

install_iceoryx() {
    echo "[iceoryx] Installing build dependencies via apt..."
    sudo apt-get update -qq
    sudo apt-get install -y -qq gcc g++ cmake libacl1-dev libncurses5-dev pkg-config 2>&1 | tail -3

    mkdir -p "$SCRIPT_DIR/.deps"
    cd "$SCRIPT_DIR/.deps"

    if [ ! -d "iceoryx" ]; then
        echo "[iceoryx] Cloning iceoryx..."
        git clone https://github.com/eclipse-iceoryx/iceoryx.git
    fi

    cd iceoryx
    echo "[iceoryx] Checking out $ICEORYX_VERSION..."
    git fetch --tags 2>/dev/null || true
    git checkout "$ICEORYX_VERSION" 2>/dev/null || {
        echo "[iceoryx] Failed to checkout $ICEORYX_VERSION, trying with fetch --depth 1..."
        git fetch --depth 1 origin tag "$ICEORYX_VERSION" 2>&1 | tail -3
        git checkout "$ICEORYX_VERSION"
    }

    echo "[iceoryx] Configuring $ICEORYX_VERSION..."
    rm -rf build
    cmake -Bbuild -Hiceoryx_meta \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TEST=OFF \
        -DEXAMPLES=OFF \
        -DBINDING_C=OFF \
        -DINTROSPECTION=OFF \
        -DCMAKE_INSTALL_PREFIX="$ICEORYX_INSTALL_DIR" \
        2>&1 | tail -5

    echo "[iceoryx] Building $ICEORYX_VERSION (this may take a few minutes)..."
    cmake --build build -j$(nproc) 2>&1 | tail -5

    rm -rf "$ICEORYX_INSTALL_DIR"
    echo "[iceoryx] Installing to $ICEORYX_INSTALL_DIR..."
    cmake --build build --target install 2>&1 | tail -3

    cd "$SCRIPT_DIR"
    echo "[iceoryx] iceoryx $ICEORYX_VERSION installed successfully"
}

install_bins() {
    mkdir -p "$HOME/.local/bin"
    cp src/rl_sar/build/bin/ares src/rl_sar/build/bin/ares_driver_node "$HOME/.local/bin/"
    echo "Installed: ~/.local/bin/ares, ~/.local/bin/ares_driver_node"
}

ensure_iceoryx

if [ -d "$ICEORYX_INSTALL_DIR/lib64" ]; then
    export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:+$CMAKE_PREFIX_PATH:}$ICEORYX_INSTALL_DIR/lib64"
fi
export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:+$CMAKE_PREFIX_PATH:}$ICEORYX_INSTALL_DIR"

if [ "$ACTION" = "full" ]; then
    echo "=== ARES Full Build ==="

    echo ""
    echo "[1/2] Building driver..."
    cmake -S driver -B driver/build -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3
    cmake --build driver/build --target dog_driver collect_pace_data -j$(nproc) 2>&1 | tail -3
    cp -u driver/build/libdog_driver.so driver/libdog_driver.so 2>/dev/null || true

    echo ""
    echo "[2/2] Building iceoryx nodes..."
    rm -rf src/rl_sar/build
    cmake -S src/rl_sar -B src/rl_sar/build -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3
    cmake --build src/rl_sar/build --target ares ares_driver_node -j$(nproc) 2>&1 | tail -3

    install_bins
    echo "=== Done ==="

elif [ "$ACTION" = "make" ]; then
    echo "=== ARES Incremental Build ==="

    if [ ! -f driver/build/Makefile ]; then
        cmake -S driver -B driver/build -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3
    fi
    cmake --build driver/build --target dog_driver collect_pace_data -j$(nproc) 2>&1 | tail -3
    cp -u driver/build/libdog_driver.so driver/libdog_driver.so 2>/dev/null || true

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
