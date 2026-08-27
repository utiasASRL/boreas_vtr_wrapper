## Runs a whole batch of mapping/localization "groupings" in one shot, so you
## don't have to keep re-invoking run_parallel_test.sh by hand for odometry
## and then again for localization, per group.
##
## For each grouping below, this script:
##   1. Runs odometry (teach) on the mapping sequence
##   2. Runs localization (repeat) on every other sequence in that grouping,
##      against the map built in step 1
## All odometry jobs (across all groupings) run together as one parallel
## batch, then all localization jobs (across all groupings) run together as
## a second parallel batch -- this is just run_parallel_test.sh's odometry
## and localization phases, generalized to many mapping sequences at once.
## Evaluation runs at the end, once per phase.
##
## This script assumes the following environment variables are set:
##   ROOTDIR VTRRROOT VTRRDATA VTRRESULT
## These are all set automatically using the setup_container.sh script!
##
## example usage: bash runtime/run_all_boreas_tests.sh radar
##                bash runtime/run_all_boreas_tests.sh radar --groupsize 4
##                bash runtime/run_all_boreas_tests.sh radar --skip-eval

set -u

# --------------------------------------------------------------------------
# USER INPUT: define your mapping/localization groupings here.
# Key   = the mapping (teach/odometry) sequence for the group
# Value = space-separated list of localization (repeat) sequences to run
#         against that group's map
# --------------------------------------------------------------------------
declare -A SEQ_GROUPS
# Tunnel
SEQ_GROUPS["boreas-2024-12-04-14-28"]="boreas-2024-12-04-14-38 boreas-2024-12-04-14-50 boreas-2024-12-04-15-04 boreas-2024-12-04-15-19"
# SEQ_GROUPS["boreas-2024-12-04-14-44"]="boreas-2024-12-04-14-34 boreas-2024-12-04-14-59 boreas-2024-12-04-15-10 boreas-2024-12-04-15-24"

# Suburbs
SEQ_GROUPS["boreas-2024-12-03-12-54"]="boreas-2024-12-05-14-25 boreas-2025-01-08-10-59 boreas-2025-01-08-11-22"

# Farm
SEQ_GROUPS["boreas-2025-07-18-14-55"]="boreas-2025-07-18-15-12 boreas-2025-07-18-15-30 boreas-2025-07-18-15-48"

# Regional
SEQ_GROUPS["boreas-2024-12-03-13-13"]="boreas-2024-12-10-12-07 boreas-2024-12-10-12-38"
# SEQ_GROUPS["boreas-2024-12-03-13-34"]="boreas-2024-12-10-12-24 boreas-2024-12-10-12-56"

# Skyway
SEQ_GROUPS["boreas-2024-12-04-11-45"]="boreas-2024-12-04-11-56 boreas-2024-12-04-12-08 boreas-2024-12-04-12-19"

# Industrial
SEQ_GROUPS["boreas-2024-12-05-14-12"]="boreas-2024-12-23-16-27 boreas-2024-12-23-16-44 boreas-2024-12-23-17-01"

# --------------------------------------------------------------------------
# Args
# --------------------------------------------------------------------------
SENSOR=$1       # [radar, lidar, radar_lidar, aeva]
shift || true

GROUPSIZE=1
SKIP_EVAL=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --groupsize)
            GROUPSIZE="$2"
            shift 2
            ;;
        --skip-eval)
            SKIP_EVAL=1
            shift
            ;;
        *)
            echo "Warning: Unknown argument $1 (ignored)"
            shift
            ;;
    esac
done

if [ -z "${SENSOR:-}" ]; then
    echo "Usage: bash run_all_boreas_tests.sh <radar|lidar|radar_lidar|aeva> [--groupsize N] [--skip-eval]"
    exit 1
fi

TEST_SCRIPT="${VTRRROOT}/runtime/run_test.sh"
EVAL_SCRIPT="${VTRRROOT}/runtime/run_multi_eval.sh"

export VTRRRESULT=${VTRRESULT}/${SENSOR}
mkdir -p ${VTRRRESULT}

MAPPING_SEQS=("${!SEQ_GROUPS[@]}")

if [ ${#MAPPING_SEQS[@]} -eq 0 ]; then
    echo "Error: SEQ_GROUPS is empty -- add at least one mapping sequence to the SEQ_GROUPS array."
    exit 1
fi

echo "=========================================================="
echo "Sensor:     ${SENSOR}"
echo "Groupsize:  ${GROUPSIZE} (max parallel jobs per phase)"
echo "Groups to run (${#MAPPING_SEQS[@]}):"
for m in "${MAPPING_SEQS[@]}"; do
    echo "  ${m}  ->  ${SEQ_GROUPS[$m]}"
done
echo "=========================================================="

# --------------------------------------------------------------------------
# Runs a batch of "label|command" entries with at most GROUPSIZE running at
# once, then waits for all of them and reports pass/fail per job.
# --------------------------------------------------------------------------
run_batch () {
    local -n _batch=$1
    local pids=()
    local labels=()
    local running=0

    for entry in "${_batch[@]}"; do
        local label="${entry%%|*}"
        local cmd="${entry#*|}"
        echo "[${label}] launching: ${cmd}"
        eval "${cmd}" &>/dev/null &
        pids+=($!)
        labels+=("${label}")
        running=$((running + 1))
        if (( running >= GROUPSIZE )); then
            wait -n
            running=$((running - 1))
        fi
    done

    local failed=0
    for i in "${!pids[@]}"; do
        if wait "${pids[$i]}"; then
            echo "[${labels[$i]}] finished OK"
        else
            echo "[${labels[$i]}] FAILED"
            failed=1
        fi
    done
    return ${failed}
}

# --------------------------------------------------------------------------
# Phase 1: odometry on every mapping sequence
# --------------------------------------------------------------------------
echo "=========================================================="
echo "Phase 1/2: odometry on mapping sequences"
echo "=========================================================="
ODO_JOBS=()
for m in "${MAPPING_SEQS[@]}"; do
    ODO_JOBS+=("${m}|bash ${TEST_SCRIPT} odometry ${SENSOR} ${m}")
done
run_batch ODO_JOBS
ODO_RC=$?

# --------------------------------------------------------------------------
# Phase 2: localization on every (mapping, loc) pair, across all groups
# --------------------------------------------------------------------------
echo "=========================================================="
echo "Phase 2/2: localization on repeat sequences"
echo "=========================================================="
LOC_JOBS=()
for m in "${MAPPING_SEQS[@]}"; do
    for l in ${SEQ_GROUPS[$m]}; do
        LOC_JOBS+=("${m}::${l}|bash ${TEST_SCRIPT} localization ${SENSOR} ${m} ${l}")
    done
done
run_batch LOC_JOBS
LOC_RC=$?

# --------------------------------------------------------------------------
# Evaluation: odometry is evaluated once, over all mapping sequences at once.
# Localization is evaluated once PER GROUP, so each group gets its own
# multi_loc_eval.py report instead of one report averaged across groups.
# --------------------------------------------------------------------------
if [ ${SKIP_EVAL} -eq 0 ]; then
    echo "=========================================================="
    echo "Evaluating odometry results"
    echo "=========================================================="
    bash ${EVAL_SCRIPT} odometry ${SENSOR} --seq "${MAPPING_SEQS[@]}"

    echo "=========================================================="
    echo "Evaluating localization results (per group)"
    echo "=========================================================="
    for m in "${MAPPING_SEQS[@]}"; do
        echo "----------------------------------------------------------"
        echo "Group: ${m}  ->  ${SEQ_GROUPS[$m]}"
        echo "----------------------------------------------------------"
        bash ${EVAL_SCRIPT} localization ${SENSOR} --seq "${m}"
    done
else
    echo "Skipping evaluation (--skip-eval)"
fi

echo "=========================================================="
echo "Done. Results in: ${VTRRRESULT}"
if [ ${ODO_RC} -ne 0 ] || [ ${LOC_RC} -ne 0 ]; then
    echo "One or more test jobs FAILED -- check the log above."
    exit 1
fi
echo "=========================================================="
