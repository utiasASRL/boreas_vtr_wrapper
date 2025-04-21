## This script assumes the following environment variables are set:
##   ROOTDIR VTRRROOT VTRRDATA VTRRESULT
## These are all set automatically using the setup_container.sh script!
## example usage: bash run_parallel_test.sh localization radar

# USER INPUT: SELECT THE SEQUENCES YOU WISH TO TEST IN PARALLEL FOR EITHER MODE
if [ "$1" = "odometry" ]; then
    # Odometry sequences, SET THESE YOURSELF
    SEQUENCES=(
    #'boreas-2020-11-26-13-58'
    #'boreas-2020-12-04-14-00'
    #'boreas-2021-01-26-10-59'
    #'boreas-2021-02-09-12-55'
    #'boreas-2021-03-09-14-23'
    #'boreas-2021-06-29-18-53'
    #'boreas-2021-09-08-21-00'

    # # Tunnel
    # " boreas-2024-02-29-14-39 "
    # " boreas-2024-02-29-14-53 "
    # " boreas-2024-02-29-15-02 "
    # " boreas-2024-02-29-15-11 "
    # # Skyway
    # " boreas-2024-02-29-11-54 "
    # " boreas-2024-02-29-12-13 "
    # " boreas-2024-02-29-12-31 "
    # " boreas-2024-02-29-12-48 "
    # # Hwy7
    # " boreas-2024-01-23-12-15 "
    # " boreas-2024-01-23-12-32 "
    # " boreas-2024-02-13-16-13 "
    # " boreas-2024-03-08-12-27 "
    # # Glen Shields
    # " boreas-2024-01-09-14-00 "
    # " boreas-2024-01-25-11-44 "
    # " boreas-2024-02-13-15-26 "
    # " boreas-2024-02-21-12-36 "

    # "boreas-2024-11-02-14-14"
    # "boreas-2024-11-02-14-44"
    # "boreas-2024-11-04-14-13"
    # "boreas-2024-11-04-14-32"

    # New Data
    "boreas-2024-12-03-10-24" # glen
    "boreas-2024-12-03-12-54" # glen

    "boreas-2024-12-03-13-13" # hwy 7 up
    "boreas-2024-12-03-13-34" # hwy 7 down
    "boreas-2024-12-10-12-07" # hwy 7 up
    "boreas-2024-12-10-12-24" # hwy 7 down
    "boreas-2024-12-10-12-38" # hwy 7 up
    "boreas-2024-12-10-12-56" # hwy 7 down

    "boreas-2024-12-04-11-45" # skyway
    "boreas-2024-12-04-11-56" # skyway
    "boreas-2024-12-04-12-08" # skyway
    "boreas-2024-12-04-12-19" # skyway
    "boreas-2024-12-04-12-34" # skyway

    "boreas-2024-12-04-14-28" # tunnel up
    "boreas-2024-12-04-14-34" # tunnel down
    "boreas-2024-12-04-14-38" # tunnel up
    "boreas-2024-12-04-14-44" # tunnel down
    "boreas-2024-12-04-14-50" # tunnel up
    # "boreas-2024-12-04-14-59" # tunnel down
    # "boreas-2024-12-04-15-04" # tunnel up
    # "boreas-2024-12-04-15-10" # tunnel down
    # "boreas-2024-12-04-15-19" # tunnel up
    # "boreas-2024-12-04-15-24" # tunnel down

    )
else
    # Odometry reference for localization, SET THIS YOURSELF
    REFERENCE='boreas-2020-11-26-13-58'
    # Localization sequences, SET THESE YOURSELF
    SEQUENCES=(
    # Train sequences
    #'boreas-2020-12-01-13-26'
    #'boreas-2020-12-18-13-44'
    #'boreas-2021-02-02-14-07'
    #'boreas-2021-03-02-13-38'
    #'boreas-2021-03-30-14-23'
    #'boreas-2021-04-08-12-44'
    #'boreas-2021-04-20-14-11'
    #'boreas-2021-04-29-15-55'
    #'boreas-2021-05-06-13-19'
    #'boreas-2021-06-17-17-52'
    #'boreas-2021-08-05-13-34'
    #'boreas-2021-09-07-09-35'

    # Validation sequences
    #'boreas-2021-01-15-12-17'
    #'boreas-2021-03-23-12-43'
    #'boreas-2021-04-13-14-49'
    #'boreas-2021-06-03-16-00'

    
    # Test sequences
    #'boreas-2020-12-04-14-00'
    #'boreas-2021-01-26-10-59'
    #'boreas-2021-02-09-12-55'
    #'boreas-2021-03-09-14-23'
    #'boreas-2021-06-29-18-53'
    #'boreas-2021-09-08-21-00'
    )

    # Sequences that didnt work:
    # 'boreas-2021-05-13-16-11' # Doesnt converge
    # 'boreas-2021-01-19-15-08' # Malformed graph? But error is good, maybe worth retrying
fi

# Get arguments
MODE=$1         # [odometry, localization]
SENSOR=$2       # [radar, lidar, radar_lidar]

# Set results subfolder, VTRRESULT is set in setup_container.sh
export VTRRRESULT=${VTRRESULT}/${SENSOR}

EVAL_SCRIPT="${ROOTDIR}/runtime/run_eval.sh"

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
