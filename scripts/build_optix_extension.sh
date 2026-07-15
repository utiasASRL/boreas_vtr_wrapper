#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ROOTDIR=${ROOTDIR:-$(dirname "$SCRIPT_DIR")}
SOURCE_DIR=$ROOTDIR/native/optix_range_tracer
BUILD_DIR=${OPTIX_EXTENSION_BUILD_DIR:-$ROOTDIR/build/optix_range_tracer}
OPTIX_SDK_ROOT=${OPTIX_SDK_ROOT:-/optix_sdk}
OVERLAY=$BUILD_DIR/sdk-overlay
PYTHON=${PYTHON_EXECUTABLE:-$(command -v python)}

[[ -f $OPTIX_SDK_ROOT/include/optix.h ]] || { echo "Invalid OPTIX_SDK_ROOT: $OPTIX_SDK_ROOT" >&2; exit 1; }
[[ -f $OPTIX_SDK_ROOT/SDK/CMakeLists.txt ]] || { echo "OptiX SDK samples are required." >&2; exit 1; }
command -v nvidia-smi >/dev/null || { echo "nvidia-smi is unavailable." >&2; exit 1; }
command -v nvcc >/dev/null || { echo "nvcc is unavailable." >&2; exit 1; }

driver=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -n1)
driver_major=${driver%%.*}
version=$(awk '/^#define OPTIX_VERSION / { print $3; exit }' "$OPTIX_SDK_ROOT/include/optix.h")
optix_major=$((version / 10000))
optix_minor=$(((version % 10000) / 100))
echo "Driver $driver, OptiX SDK $optix_major.$optix_minor"

if (( optix_major >= 9 && driver_major < 560 )); then
    echo "OptiX $optix_major.$optix_minor is not supported by R$driver_major; use OptiX 8 on this host." >&2
    exit 1
fi
if (( optix_major == 8 && driver_major < 535 )); then
    echo "OptiX 8 requires an R535-or-newer driver." >&2
    exit 1
fi

# Use NVIDIA's sample build rules without modifying the mounted SDK.
mkdir -p "$OVERLAY/SDK"
for path in "$OPTIX_SDK_ROOT"/*; do
    name=$(basename "$path")
    [[ $name == SDK ]] || ln -sfn "$path" "$OVERLAY/$name"
done
for path in "$OPTIX_SDK_ROOT/SDK"/*; do
    name=$(basename "$path")
    [[ $name == optixTriangle ]] || ln -sfn "$path" "$OVERLAY/SDK/$name"
done
ln -sfn "$SOURCE_DIR" "$OVERLAY/SDK/optixTriangle"

torch_prefix=$($PYTHON -c 'import torch; print(torch.utils.cmake_prefix_path)')
cmake -S "$OVERLAY/SDK" -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$torch_prefix" \
    -DOptiX_INSTALL_DIR="$OPTIX_SDK_ROOT" \
    -DOPTIX_BUILD_PYTORCH_BINDING=ON \
    -DPython3_EXECUTABLE="$PYTHON"
cmake --build "$BUILD_DIR" --target optix_range_tracer -j"${BUILD_JOBS:-$(nproc)}"

"$BUILD_DIR/bin/optixTriangle" --dim=61x61 --poses=400
PYTHONPATH="$BUILD_DIR/python${PYTHONPATH:+:$PYTHONPATH}" \
    "$PYTHON" "$SOURCE_DIR/test_optix_torch.py"
echo "OptiX extension ready: $BUILD_DIR/python/optix_range_tracer.so"
