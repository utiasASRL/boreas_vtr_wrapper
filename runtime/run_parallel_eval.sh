## This script assumes the following environment variables are set:
##   ROOTDIR VTRRROOT VTRRDATA VTRRESULT
## These are all set automatically using the setup_container.sh script!
## example usage: bash run_parallel_test.sh localization radar

# USER INPUT: SELECT THE SEQUENCES YOU WISH TO TEST IN PARALLEL FOR EITHER MODE
if [ "$1" = "odometry" ]; then
    # Odometry sequences, SET THESE YOURSELF
    SEQUENCES=(
    # "boreas-2024-12-03-10-24" # glen, aeva, CALIB
    # "boreas-2024-12-03-12-54" # glen, aeva
    # "boreas-2024-12-05-14-25" # glen, aeva
    # "boreas-2025-01-08-10-59" # glen, aeva
    # "boreas-2025-01-08-11-22" # glen, aeva
    # "boreas-2025-01-08-12-28" # glen, aeva
    # "boreas-2025-02-15-16-58" # glen
    # "boreas-2025-02-15-17-19" # glen
    # "boreas-2025-02-21-14-51" # glen
    # "boreas-2025-02-22-11-32" # glen
    # "boreas-2025-02-22-12-26" # glen
    # "boreas-2025-09-22-13-16" # glen, CALIB

    # "boreas-2024-12-03-13-13" # hwy 7 up, calib
    # "boreas-2024-12-03-13-34" # hwy 7 down, calib
    # "boreas-2024-12-10-12-07" # hwy 7 up
    # "boreas-2024-12-10-12-24" # hwy 7 down
    # "boreas-2024-12-10-12-38" # hwy 7 up
    # "boreas-2024-12-10-12-56" # hwy 7 down

    # "boreas-2024-12-04-11-45" # skyway, calib
    # "boreas-2024-12-04-11-56" # skyway
    # "boreas-2024-12-04-12-08" # skyway
    # "boreas-2024-12-04-12-19" # skyway
    # "boreas-2024-12-04-12-34" # skyway

    # "boreas-2024-12-04-14-28" # tunnel up, aeva
    # "boreas-2024-12-04-14-34" # tunnel down, aeva
    # "boreas-2024-12-04-14-38" # tunnel up, aeva
    # "boreas-2024-12-04-14-44" # tunnel down, aeva
    # "boreas-2024-12-04-14-50" # tunnel up, aeva
    # "boreas-2024-12-04-14-59" # tunnel down, aeva
    # "boreas-2024-12-04-15-04" # tunnel up, aeva
    # "boreas-2024-12-04-15-10" # tunnel down, aeva
    # "boreas-2024-12-04-15-19" # tunnel up, aeva
    # "boreas-2024-12-04-15-24" # tunnel down, aeva

    # Commerical
    # "boreas-2024-12-05-14-12" # commercial, aeva
    # "boreas-2024-12-23-16-27" # commercial, aeva
    # "boreas-2024-12-23-16-44" # commercial, aeva
    # "boreas-2024-12-23-17-01" # commercial, aeva
    # "boreas-2024-12-23-17-18" # commercial, aeva

    # UTIAS
    # "boreas-2025-02-15-15-58" # utias
    # "boreas-2025-02-15-16-08" # utias
    # "boreas-2025-02-15-16-16" # utias
    # "boreas-2025-02-15-16-25" # utias
    # "boreas-2025-02-15-16-33" # utias
    # "boreas-2025-02-22-11-24" # utias
    # "boreas-2025-02-22-11-52" # utias
    # "boreas-2025-02-22-12-01" # utias
    # "boreas-2025-02-22-12-09" # utias
    # "boreas-2025-02-22-12-18" # utias

    # Forest
    # "boreas-2025-07-18-10-00" # forest
    # "boreas-2025-07-18-10-33" # forest
    # "boreas-2025-07-18-11-00" # forest
    # "boreas-2025-07-18-11-25" # forest
    # "boreas-2025-07-18-11-53" # forest

    # Farm Field
    # "boreas-2025-07-18-14-55" # farm field
    # "boreas-2025-07-18-15-12" # farm field
    # "boreas-2025-07-18-15-30" # farm field
    # "boreas-2025-07-18-15-48" # farm field
    # "boreas-2025-07-18-16-05" # farm field
    # "boreas-2025-08-13-09-01" # farm field
    # "boreas-2025-08-13-09-21" # farm field
    # "boreas-2025-08-13-09-46" # farm field
    # "boreas-2025-08-13-10-12" # farm field
    # "boreas-2025-08-13-10-36" # farm field

    # Freeway
    # "boreas-2025-07-18-16-24" # freeway south
    # "boreas-2025-08-13-07-54" # freeway north
    # "boreas-2025-08-13-11-52" # freeway south

    # Urban Canyon
    # "boreas-2025-08-06-06-33" # urban canyon
    # "boreas-2025-08-06-07-05" # urban canyon
    # "boreas-2025-08-06-07-41" # urban canyon
    # "boreas-2025-08-06-08-35" # urban canyon
    # "boreas-2025-08-06-10-48" # urban canyon
    # "boreas-2025-08-06-11-32" # urban canyon
    # "boreas-2025-08-06-12-20" # urban canyon
    )
else
    # Odometry reference for localization, SET THIS YOURSELF
    # REFERENCE="boreas-2024-12-03-12-54" # glen
    # REFERENCE="boreas-2024-12-03-13-13" # hwy 7, up
    # REFERENCE="boreas-2024-12-03-13-34"  # hwy 7, down
    # REFERENCE="boreas-2024-12-04-11-45" # skyway
    # REFERENCE="boreas-2024-12-04-14-28" # tunnel, up
    # REFERENCE="boreas-2024-12-04-14-34" # tunnel down
    # REFERENCE="boreas-2024-12-05-14-12" # commercial, aeva
    # REFERENCE="boreas-2025-02-22-11-24" # utias, no drift
    # REFERENCE="boreas-2025-07-18-10-00" # forest
    # REFERENCE="boreas-2025-07-18-14-55" # farm field
    # REFERENCE="boreas-2025-07-18-16-24" # freeway south
    REFERENCE="boreas-2025-08-06-06-33" # urban canyon
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
