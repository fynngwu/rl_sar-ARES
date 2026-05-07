#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INFER_DIR="${SCRIPT_DIR}/library/inference_runtime"

ONNX_VERSION="1.17.1"
LIBTORCH_VERSION="2.1.0"
LIBTORCH_CUDA="cpu"

detect_arch() {
    local arch="$(uname -m)"
    case "$arch" in
        x86_64)  echo "x64" ;;
        aarch64) echo "aarch64" ;;
        *)       echo "UNKNOWN" ;;
    esac
}

download_onnx() {
    local ONNX_INCLUDE="/usr/local/include"
    local ONNX_LIB="/usr/local/lib"
    local HEADER="${ONNX_INCLUDE}/onnxruntime_cxx_api.h"
    local LIB="${ONNX_LIB}/libonnxruntime.so"

    if [ -f "${HEADER}" ] && [ -f "${LIB}" ]; then
        echo "[ONNX] Already installed in /usr/local"
        return 0
    fi

    if [ "$EUID" -ne 0 ]; then
        echo "[ONNX] Requires sudo to install to /usr/local"
        exec sudo -- "$0" onnx
    fi

    local arch="$(detect_arch)"
    if [ "$arch" = "UNKNOWN" ]; then
        echo "[ONNX] ERROR: Unsupported architecture $(uname -m)"
        return 1
    fi

    local PACKAGE="onnxruntime-linux-${arch}-${ONNX_VERSION}"
    local URL="https://github.com/microsoft/onnxruntime/releases/download/v${ONNX_VERSION}/${PACKAGE}.tgz"
    local TMPDIR="$(mktemp -d)"

    echo "[ONNX] Downloading ${PACKAGE}..."
    curl -fSL -o "${TMPDIR}/${PACKAGE}.tgz" "${URL}"

    echo "[ONNX] Extracting..."
    tar xzf "${TMPDIR}/${PACKAGE}.tgz" -C "${TMPDIR}"

    echo "[ONNX] Installing headers to ${ONNX_INCLUDE}..."
    cp -n "${TMPDIR}/${PACKAGE}/include/"*.h "${ONNX_INCLUDE}/"

    echo "[ONNX] Installing libraries to ${ONNX_LIB}..."
    cp -n "${TMPDIR}/${PACKAGE}/lib/"*.so* "${ONNX_LIB}/"
    ldconfig

    rm -rf "${TMPDIR}"
    echo "[ONNX] Installed to /usr/local"

    local SYMLINK_DIR="${SCRIPT_DIR}/library/inference_runtime/onnxruntime"
    mkdir -p "${SYMLINK_DIR}/include" "${SYMLINK_DIR}/lib"
    for h in "${ONNX_INCLUDE}"/onnxruntime_*.h; do
        local base="$(basename "$h")"
        ln -sf "${ONNX_INCLUDE}/${base}" "${SYMLINK_DIR}/include/${base}"
    done
    for l in "${ONNX_LIB}"/libonnxruntime.so*; do
        local base="$(basename "$l")"
        ln -sf "${ONNX_LIB}/${base}" "${SYMLINK_DIR}/lib/${base}"
    done
    echo "[ONNX] Symlinks created at ${SYMLINK_DIR}"
}

download_libtorch() {
    local TORCH_DIR="${INFER_DIR}/libtorch"
    if [ -f "${TORCH_DIR}/lib/libtorch.so" ] && [ -f "${TORCH_DIR}/include/torch/torch.h" ]; then
        echo "[LibTorch] Already installed at ${TORCH_DIR}"
        return 0
    fi

    local arch="$(detect_arch)"
    if [ "$arch" != "x64" ]; then
        echo "[LibTorch] Skipped (only pre-built for x64, aarch64 needs manual build)"
        return 0
    fi

    local PACKAGE="libtorch-cxx11-abi-shared-with-deps-${LIBTORCH_VERSION}%2B${LIBTORCH_CUDA}.zip"
    local URL="https://download.pytorch.org/libtorch/${LIBTORCH_CPU}/libtorch-cxx11-abi-shared-with-deps-${LIBTORCH_VERSION}%2B${LIBTORCH_CPU}.zip"
    local URL="https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-${LIBTORCH_VERSION}%2Bcpu.zip"
    local TMPDIR="$(mktemp -d)"

    echo "[LibTorch] Downloading ${LIBTORCH_VERSION} CPU..."
    curl -fSL -o "${TMPDIR}/libtorch.zip" "${URL}"

    echo "[LibTorch] Extracting..."
    rm -rf "${TORCH_DIR}"
    unzip -q "${TMPDIR}/libtorch.zip" -d "${TMPDIR}"
    mv "${TMPDIR}/libtorch" "${TORCH_DIR}"

    rm -rf "${TMPDIR}"
    echo "[LibTorch] Installed at ${TORCH_DIR}"
}

case "${1:-all}" in
    onnx)
        download_onnx
        ;;
    libtorch)
        download_libtorch
        ;;
    all)
        download_onnx
        download_libtorch
        ;;
    *)
        echo "Usage: $0 [onnx|libtorch|all]"
        echo "  onnx     - Download ONNX Runtime ${ONNX_VERSION}"
        echo "  libtorch - Download LibTorch ${LIBTORCH_VERSION} (CPU)"
        echo "  all      - Download both (default)"
        exit 1
        ;;
esac
