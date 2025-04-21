## This script assumes the following environment variables are set:
##   ROOTDIR VTRRROOT VTRRDATA VTRRESULT
## These are all set automatically using the setup_container.sh script!
## example usage: bash run_parallel_test.sh localization radar RESULT_DIR (optional) N SEQUENCES (optional)

# Get arguments
MODE=$1         # [odometry, localization]
SENSOR=$2       # [radar, lidar, radar_lidar]
# Set results subfolder, VTRRESULT is set in setup_container.sh
if [ -z "$3" ]; then
    export VTRRRESULT=${VTRRESULT}/${SENSOR}
else
    export VTRRRESULT=${3}
fi


# USER INPUT: SELECT THE SEQUENCES YOU WISH TO TEST IN PARALLEL FOR EITHER MODE
if [ "$1" = "odometry" ]; then

    # Check if $3 exists, if not, use default
    if [ -z "$4" ]; then
        # Odometry sequences, SET THESE YOURSELF
        SEQUENCES=(
        " boreas-2024-01-09-14-00 "
        " boreas-2024-01-25-11-44 "
        " boreas-2024-02-13-15-26 "
        " boreas-2024-02-21-12-36 "
        # " boreas-2024-01-23-12-15 "
        # " boreas-2024-01-23-12-32 "
        # " boreas-2024-02-13-16-13 "
        # " boreas-2024-03-08-12-27 "
        # " boreas-2024-02-29-11-54 "
        # " boreas-2024-02-29-12-13 "
        # " boreas-2024-02-29-12-31 "
        # " boreas-2024-02-29-12-48 "
        # " boreas-2024-02-29-14-39 "
        # " boreas-2024-02-29-14-53 "
        # " boreas-2024-02-29-15-02 "
        # " boreas-2024-02-29-15-11 "
        )
    else
        SEQUENCES=()
        # Shift the arguments to get the sequences
        shift 3
        for sequence in "$@"
        do
            SEQUENCES+=($sequence)
        done
    fi
else
    # Odometry reference for localization, SET THIS YOURSELF
    REFERENCE='boreas-2020-11-26-13-58'
fi

# First, make sure expected txt files are generated
# Source the VTR environment with the testing package
source ${VTRRROOT}/src/install/setup.bash
source ${VTRROOT}/venv/bin/activate
for ODO_INPUT in ${SEQUENCES[@]}; do
    RES_DIR=${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT}
    # Log
    echo "Evaluating odometry of sequence ${ODO_INPUT}, storing result to ${RES_DIR}"

    #   - dump odometry result to boreas expected format (txt file)
    if [ ${SENSOR} == "radar" ]; then
        python ${VTRRROOT}/src/vtr_testing_radar/script/boreas_generate_odometry_result.py --dataset ${VTRRDATA} --path ${VTRRRESULT}/${ODO_INPUT} --velocity
    elif [ ${SENSOR} == "lidar" ]; then
        python ${VTRRROOT}/src/vtr_testing_lidar/script/boreas_generate_odometry_result.py --dataset ${VTRRDATA} --path ${VTRRRESULT}/${ODO_INPUT} --velocity
    fi
done

#   - evaluate the result using the evaluation script
if [ ${SENSOR} == "radar" ]; then
    python ${ROOTDIR}/doppler_radar/eval/multi_odom_eval.py --gt ${VTRRDATA} --pred ${VTRRRESULT} --radar --velocity --sequences ${SEQUENCES[@]}
elif [ ${SENSOR} == "lidar" ]; then
    python ${ROOTDIR}/doppler_radar/eval/multi_odom_eval.py --gt ${VTRRDATA} --pred ${VTRRRESULT} --velocity --sequences ${SEQUENCES[@]}
fi