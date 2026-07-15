ROOTDIR=${ROOTDIR:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}
cd "$ROOTDIR"

export VTRROOT=$ROOTDIR
export VTRRROOT=$ROOTDIR
export VTRSRC=$ROOTDIR/external/vtr3
export VTRRDATA=${VTRRDATA:-/boreas_data}
export BOREAS_OUTPUT_ROOT=${BOREAS_OUTPUT_ROOT:-$VTRRDATA}
export VTRRESULT=${VTRRESULT:-$ROOTDIR/results}
export VTRRRESULT=$VTRRESULT/radar/boreas

export OPTIX_SDK_ROOT=${OPTIX_SDK_ROOT:-/optix_sdk}
export OPTIX_EXTENSION_BUILD_DIR=${OPTIX_EXTENSION_BUILD_DIR:-$ROOTDIR/build/optix_range_tracer}
export PYTHONPATH=$OPTIX_EXTENSION_BUILD_DIR/python${PYTHONPATH:+:$PYTHONPATH}

source /opt/ros/humble/setup.bash
[[ -f $VTRSRC/main/install/setup.bash ]] && source "$VTRSRC/main/install/setup.bash"
[[ -f $VTRRROOT/src/install/setup.bash ]] && source "$VTRRROOT/src/install/setup.bash"

mkdir -p "$VTRRESULT" "$BOREAS_OUTPUT_ROOT"

if [[ -f $ROOTDIR/venv/bin/activate ]]; then
    source "$ROOTDIR/venv/bin/activate"
else
    echo "Python environment not found; run scripts/create_venv.sh."
fi

# export OMP_NUM_THREADS=6
# export NEPTUNE_API_TOKEN=temp
