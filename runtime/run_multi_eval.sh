## This script assumes the following environment variables are set:
##   ROOTDIR VTRRROOT VTRRDATA VTRRESULT
## These are all set automatically using the setup_container.sh script!
## example usage: bash run_parallel_test.sh localization radar RESULT_DIR (optional) N SEQUENCES (optional)

# # Get arguments
# MODE=$1         # [odometry, localization]
# SENSOR=$2       # [radar, lidar, radar_lidar]
# # Set results subfolder, VTRRESULT is set in setup_container.sh
# if [ -z "$3" ]; then
#     export VTRRRESULT=${VTRRESULT}/${SENSOR}
# else
#     export VTRRRESULT=${3}
# fi
MODE="$1"        # odometry | localization
SENSOR="$2"      # radar | lidar | radar_lidar

# Remove the two required args from $@
shift 2

# --------------------------------------
# Default values
# --------------------------------------
RESULT_DIR=""     # Will be overridden by --result_dir
SEQUENCES=()

# --------------------------------------
# Parse optional flags in any order
# --------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --result_dir)
            RESULT_DIR="$2"
            shift 2
            ;;
        --seq)
            shift
            while [[ $# -gt 0 && "$1" != --* ]]; do
                SEQUENCES+=("$1")
                shift
            done
            ;;
        *)
            echo "Warning: Unknown argument $1 (ignored)"
            shift
            ;;
    esac
done


if [ -z "$RESULT_DIR" ]; then
    export VTRRRESULT="${VTRRESULT}/${SENSOR}"
else
    export VTRRRESULT="${RESULT_DIR}"
fi


# If no --seq flag was provided, fall back to defaults
if [ "$MODE" = "odometry" ] && [ ${#SEQUENCES[@]} -eq 0 ]; then
    SEQUENCES=(
        # Calibration
        # "boreas-2025-02-22-12-26" # glen, calib
        # "boreas-2024-12-10-12-56" # hwy 7 down, calib
        # "boreas-2024-12-04-12-34" # skyway, calib
        # "boreas-2024-12-04-15-24" # tunnel down, aeva , calib
        # "boreas-2024-12-23-17-18" # commercial, aeva , calib
        # "boreas-2025-07-18-11-53" # forest, calib
        # "boreas-2025-08-13-10-36" # farm field, calib
        # "boreas-2025-08-06-12-20" # urban canyon, calib

        # Suburbs
        # "boreas-2024-12-03-12-54" # glen, aeva
        # "boreas-2024-12-05-14-25" # glen, aeva
        # "boreas-2025-01-08-10-59" # glen, aeva
        # "boreas-2025-01-08-11-22" # glen, aeva
        # "boreas-2025-01-08-12-28" # glen, aeva
        # "boreas-2025-02-15-16-58" # glen
        # "boreas-2025-02-15-17-19" # glen
        # "boreas-2025-02-21-14-51" # glen
        # "boreas-2025-02-22-11-32" # glen

        # Hwy 7
        # "boreas-2024-12-03-13-13" # hwy 7 up
        # "boreas-2024-12-03-13-34" # hwy 7 down
        # "boreas-2024-12-10-12-07" # hwy 7 up
        # "boreas-2024-12-10-12-24" # hwy 7 down
        # "boreas-2024-12-10-12-38" # hwy 7 up

        # Tunnel
        # "boreas-2024-12-04-14-28" # tunnel up, aeva
        # "boreas-2024-12-04-14-34" # tunnel down, aeva
        # "boreas-2024-12-04-14-38" # tunnel up, aeva
        # "boreas-2024-12-04-14-44" # tunnel down, aeva
        # "boreas-2024-12-04-14-50" # tunnel up, aeva
        # "boreas-2024-12-04-14-59" # tunnel down, aeva
        # "boreas-2024-12-04-15-04" # tunnel up, aeva
        # "boreas-2024-12-04-15-10" # tunnel down, aeva
        # "boreas-2024-12-04-15-19" # tunnel up, aeva

        # Skyway
        # "boreas-2024-12-04-11-45" # skyway
        # "boreas-2024-12-04-11-56" # skyway
        # "boreas-2024-12-04-12-08" # skyway
        # "boreas-2024-12-04-12-19" # skyway

        # # Industrial
        # "boreas-2024-12-05-14-12" # industrial, aeva
        # "boreas-2024-12-23-16-27" # industrial, aeva
        # "boreas-2024-12-23-16-44" # industrial, aeva
        # "boreas-2024-12-23-17-01" # industrial, aeva

        # # Forest
        # "boreas-2025-07-18-10-00" # forest
        # "boreas-2025-07-18-10-33" # forest
        # "boreas-2025-07-18-11-00" # forest
        # "boreas-2025-07-18-11-25" # forest

        # # Farm Field
        # "boreas-2025-07-18-14-55" # farm field
        # "boreas-2025-07-18-15-12" # farm field
        # "boreas-2025-07-18-15-30" # farm field
        # "boreas-2025-07-18-15-48" # farm field
        # "boreas-2025-07-18-16-05" # farm field
        # "boreas-2025-08-13-09-01" # farm field
        # "boreas-2025-08-13-09-21" # farm field
        # "boreas-2025-08-13-09-46" # farm field
        # "boreas-2025-08-13-10-12" # farm field

        # Freeway
        # "boreas-2025-07-18-16-24" # freeway south
        # "boreas-2025-08-13-07-54" # freeway north
        # "boreas-2025-08-13-11-52" # freeway south

        # Urban Canyon
        "boreas-2025-08-06-06-33" # urban canyon
        "boreas-2025-08-06-07-05" # urban canyon
        "boreas-2025-08-06-07-41" # urban canyon
        "boreas-2025-08-06-08-35" # urban canyon
        "boreas-2025-08-06-10-48" # urban canyon
        "boreas-2025-08-06-11-32" # urban canyon
        "boreas-2025-08-06-12-20" # urban canyon
    )
fi

# First, make sure expected txt files are generated
# Source the VTR environment with the testing package
source ${VTRRROOT}/src/install/setup.bash
source ${VTRROOT}/venv/bin/activate
for ODO_INPUT in ${SEQUENCES[@]}; do
    RES_DIR=${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT}
    # echo "Generating odometry result for sequence: ${ODO_INPUT} in ${RES_DIR}"
    # Dump odometry result to boreas expected format (txt file)
    python ${VTRRROOT}/src/vtr_testing_${SENSOR}/script/boreas_generate_odometry_result.py --dataset ${VTRRDATA} --path ${VTRRRESULT}/${ODO_INPUT} --velocity --quiet
done

# Evaluate the result using the evaluation script
if [ ${SENSOR} == "radar" ]; then
    python ${ROOTDIR}/runtime/multi_odom_eval.py --gt ${VTRRDATA} --pred ${VTRRRESULT} --radar --velocity --sequences ${SEQUENCES[@]}
elif [ ${SENSOR} == "lidar" ]; then
    python ${ROOTDIR}/runtime/multi_odom_eval.py --gt ${VTRRDATA} --pred ${VTRRRESULT} --velocity --sequences ${SEQUENCES[@]}
fi