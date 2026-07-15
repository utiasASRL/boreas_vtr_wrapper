#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
export ROOTDIR=$(dirname "$SCRIPT_DIR")
PATHS_FILE=${BOREAS_PATHS_FILE:-$SCRIPT_DIR/paths.env}

if [[ ! -f "$PATHS_FILE" ]]; then
    echo "Missing $PATHS_FILE; copy scripts/paths.env.example and edit it." >&2
    exit 1
fi

source "$PATHS_FILE"
: "${BOREAS_DATA_HOST:?Set BOREAS_DATA_HOST in $PATHS_FILE}"
: "${BOREAS_OUTPUT_HOST:?Set BOREAS_OUTPUT_HOST in $PATHS_FILE}"
: "${OPTIX_SDK_HOST:?Set OPTIX_SDK_HOST in $PATHS_FILE}"

[[ -d "$BOREAS_DATA_HOST" ]] || { echo "Missing data directory: $BOREAS_DATA_HOST" >&2; exit 1; }
[[ -d "$OPTIX_SDK_HOST" ]] || { echo "Missing OptiX SDK: $OPTIX_SDK_HOST" >&2; exit 1; }
mkdir -p "$BOREAS_OUTPUT_HOST"

container_name=boreas_wrapper_$(whoami)
container_state=$(docker inspect -f '{{.State.Running}}' "$container_name" 2>/dev/null || true)

if [[ "$container_state" == true ]]; then
    echo 'Container already running, joining it now.'
    docker exec -it "$container_name" /entrypoint.sh
else
    echo 'New container run initialized.'
    docker run -it --rm --name "$container_name" \
        --gpus "${GPU_DEVICES:-all}" \
        --privileged \
        --network=host \
        --ipc=host \
        -e "DISPLAY=${DISPLAY:-}" \
        -e "ROOTDIR=$ROOTDIR" \
        -e VTRRDATA=/boreas_data \
        -e BOREAS_OUTPUT_ROOT=/boreas_output \
        -e "VTRRESULT=$ROOTDIR/results" \
        -e OPTIX_SDK_ROOT=/optix_sdk \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$ROOTDIR:$ROOTDIR:rw" \
        -v "$BOREAS_DATA_HOST:/boreas_data:ro" \
        -v "$BOREAS_OUTPUT_HOST:/boreas_output:rw" \
        -v "$OPTIX_SDK_HOST:/optix_sdk:ro" \
        -w "$ROOTDIR" "boreas_wrapper_$(whoami)"
fi
