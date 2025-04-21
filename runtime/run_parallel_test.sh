## This script assumes the following environment variables are set:
##   ROOTDIR VTRRROOT VTRRDATA VTRRESULT
## These are all set automatically using the setup_container.sh script!
## example usage: run_parallel_test.sh localization radar

# USER INPUT: SELECT THE SEQUENCES YOU WISH TO TEST IN PARALLEL FOR EITHER MODE
if [ "$1" = "odometry" ]; then
    # Odometry sequences, SET THESE YOURSELF
    SEQUENCES=(
    'boreas-2023-02-15-19-49'
    )
else
    # Odometry reference for localization, SET THIS YOURSELF
    REFERENCE='boreas-2023-02-15-19-49'
    # Localization sequences, SET THESE YOURSELF
    SEQUENCES=(
    'boreas-2023-02-15-20-07'
    'boreas-2023-02-15-20-24'
    'boreas-2023-02-15-20-43'
    )
fi

# Get arguments
MODE=$1         # [odometry, localization]
SENSOR=$2       # [radar, lidar, radar_lidar]

# Load in param file based on sensor
PARAM_FILE=${ROOTDIR}/runtime/config/${SENSOR}_config.yaml

# Set results subfolder, VTRRESULT is set in setup_container.sh
export VTRRRESULT=${VTRRESULT}/${SENSOR}
mkdir -p ${VTRRRESULT}

# maximum number of jobs running in parallel
GROUPSIZE=4

SCRIPT="${VTRRROOT}/runtime/run_test.sh"
EVAL_SCRIPT="${VTRRROOT}/runtime/run_eval.sh"

declare -A pids

# Run tests in parallel
for seq in ${SEQUENCES[@]}; do
    if [ "$1" = "odometry" ]; then
        echo "Executing command: bash $SCRIPT $1 $2 $seq &>/dev/null &"
        ### command to execute
        bash $SCRIPT $1 $2 $seq &>/dev/null &
    else
        echo "Executing command: bash $SCRIPT $1 $2 $REFERENCE $seq &>/dev/null &"
        ### command to execute
        bash $SCRIPT $1 $2 $REFERENCE $seq &>/dev/null &
    fi

    pids[${seq}]=$!
    # wait for any pid to finish if reached group size
    if [[ ${#pids[@]} -ge ${GROUPSIZE} ]]; then
        wait -n
        pid_finished=$?
        seq_finished=${pids[${pid_finished}]}
        echo "Process ${seq_finished} finished with return code ${?}"
        unset pids[$?]
    fi
done

# Wait for all processes to finish
for key in ${!pids[@]}; do
  wait ${pids[${key}]}
  echo "Process ${key} finished with return code ${?}"
  unset pids[${key}]
done

# Evaluate results from tests
if [ "$1" = "odometry" ]; then
    for seq in ${SEQUENCES[@]}; do
    echo "Executing command: bash $EVAL_SCRIPT $1 $2 $seq"
    bash $EVAL_SCRIPT $1 $2 $seq
    done
else
    echo "Executing command: bash $EVAL_SCRIPT $1 $2 $REFERENCE"
    bash $EVAL_SCRIPT $1 $2 $REFERENCE
fi