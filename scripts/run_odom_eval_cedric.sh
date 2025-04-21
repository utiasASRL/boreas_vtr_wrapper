RESULT_DIR="cedric_results"

ROUTES=(
    "glen"
    "hwy7"
    "tunnel"
    "skyway"
)

TYPES=(
    #"B1"
    #"B2"
    #"B2_star"
    #"B3"
    "B3_star"
)

EVAL_SCRIPT="${ROOTDIR}/runtime/run_multi_eval.sh"

for route in "${ROUTES[@]}"; do
    if [[ $route == "glen" ]]; then
        SEQUENCES=(
            "boreas-2024-12-03-12-54"
            "boreas-2025-01-08-10-59"
            "boreas-2025-01-08-11-22"
            "boreas-2025-01-08-12-28"
        )

    elif [[ $route == "hwy7" ]]; then
        SEQUENCES=(
            "boreas-2024-12-10-12-07"
            "boreas-2024-12-10-12-24"
            "boreas-2024-12-10-12-38"
            "boreas-2024-12-10-12-56"
        )
    elif [[ $route == "tunnel" ]]; then
        SEQUENCES=(
            "boreas-2024-12-04-14-38"
            "boreas-2024-12-04-14-44"
            "boreas-2024-12-04-14-50"
            "boreas-2024-12-04-14-59"
        )
    elif [[ $route == "skyway" ]]; then
        SEQUENCES=(
            "boreas-2024-12-04-11-56"
            "boreas-2024-12-04-12-08"
            "boreas-2024-12-04-12-19"
            "boreas-2024-12-04-12-34"
        )
    else
        echo -e "\e[31mInvalid route: $route\e[0m"
        exit 1
    fi

    for type in "${TYPES[@]}"; do
        echo -e "\e[31mEvaluating $route odometry with $type\e[0m"
        export VTRRRESULT=${VTRRESULT}/$RESULT_DIR/$type
        if [[ $type == "B3" ]]; then
            bash $EVAL_SCRIPT "odometry" "lidar" $VTRRRESULT "${SEQUENCES[@]}"
        else 
            bash $EVAL_SCRIPT "odometry" "radar" $VTRRRESULT "${SEQUENCES[@]}"
        fi
        echo -e "\e[31mEvaluating $route odometry with $type COMPLETE\e[0m"
    done
done