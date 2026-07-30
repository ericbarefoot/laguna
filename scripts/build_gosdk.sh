#!/usr/bin/env bash
#
# Build the LMI GoSdk shared libraries for linux_x64.
#
# Why this exists: the vendor SDK drop (14400-6.5.2.5_SOFTWARE_GO_SDK) ships
# prebuilt .so files only for linux_arm64 (the sensor's own CPU) and win64 —
# lib/linux_x64/ is empty. The laguna PC is x86_64, so libkApi.so and
# libGoSdk.so have to be built from the vendor makefiles before
# laguna.scanner.gosdk can load them.
#
# Prerequisite (not installed by this script — it needs root):
#
#     sudo apt install build-essential
#
# Usage:
#     scripts/build_gosdk.sh [GO_SDK_DIR]
#
# GO_SDK_DIR defaults to ~/Downloads/14400-6.5.2.5_SOFTWARE_GO_SDK/GO_SDK,
# or $LAGUNA_GOSDK_DIR if set. On success the libraries land in
# $GO_SDK_DIR/lib/linux_x64/ — point the scanner at that directory via the
# gocator config's sdk_lib_dir, or via $LAGUNA_GOSDK_LIB_DIR.
#
set -euo pipefail

GO_SDK_DIR="${1:-${LAGUNA_GOSDK_DIR:-$HOME/Downloads/14400-6.5.2.5_SOFTWARE_GO_SDK/GO_SDK}}"
CONFIG="${CONFIG:-Release}"   # Release -> lib/linux_x64, Debug -> lib/linux_x64d

if [[ ! -d "$GO_SDK_DIR" ]]; then
    echo "error: GO_SDK directory not found: $GO_SDK_DIR" >&2
    echo "       pass it as the first argument or set LAGUNA_GOSDK_DIR." >&2
    exit 1
fi

for tool in make gcc g++ python3; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "error: '$tool' not found. Install the toolchain first:" >&2
        echo "           sudo apt install build-essential" >&2
        exit 1
    fi
done

if [[ "$(uname -m)" != "x86_64" ]]; then
    echo "warning: this machine is $(uname -m), not x86_64 — the *-Linux_X64.mk" >&2
    echo "         makefiles will cross-compile and expect a toolchain under" >&2
    echo "         /tools. Use the matching *-Linux_Arm64.mk instead." >&2
fi

JOBS="$(nproc 2>/dev/null || echo 2)"

echo "==> Building kApi (config=$CONFIG, -j$JOBS)"
make -C "$GO_SDK_DIR/Platform/kApi" -f kApi-Linux_X64.mk "config=$CONFIG" -j"$JOBS"

echo "==> Building GoSdk (config=$CONFIG, -j$JOBS)"
make -C "$GO_SDK_DIR/Gocator" -f GoSdk-Linux_X64.mk "config=$CONFIG" -j"$JOBS"

if [[ "$CONFIG" == "Release" ]]; then
    OUT_DIR="$GO_SDK_DIR/lib/linux_x64"
else
    OUT_DIR="$GO_SDK_DIR/lib/linux_x64d"
fi

echo
echo "==> Built libraries in $OUT_DIR:"
ls -la "$OUT_DIR"/libkApi.so "$OUT_DIR"/libGoSdk.so 2>/dev/null || {
    echo "error: expected libkApi.so and libGoSdk.so in $OUT_DIR but they are missing." >&2
    exit 1
}

echo
echo "Point laguna at these with either:"
echo "    export LAGUNA_GOSDK_LIB_DIR=$OUT_DIR"
echo "or the 'sdk_lib_dir' key in the gocator: section of your config YAML."
