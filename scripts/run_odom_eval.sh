RESULT_DIR="paper1_results"

ROUTES=(
    "glen"
    "hwy7"
    "tunnel"
    "skyway"
)

TYPES=(
    "B1"
    # "B2"
    # "B3"
    # "N1"
    # "N2"
    # "N3"
)

EVAL_SCRIPT="${ROOTDIR}/runtime/run_multi_eval.sh"

for route in "${ROUTES[@]}"; do
    if [[ $route == "glen" ]]; then
        SEQUENCES=(
            "boreas-2024-01-09-14-00"
            "boreas-2024-01-25-11-44"
            "boreas-2024-02-13-15-26"
            "boreas-2024-02-21-12-36"
        )

    elif [[ $route == "hwy7" ]]; then
        SEQUENCES=(
            "boreas-2024-04-09-12-47"
            "boreas-2024-01-23-12-32"
            "boreas-2024-02-13-16-13"
            "boreas-2024-03-08-12-27"
        )
    elif [[ $route == "tunnel" ]]; then
        SEQUENCES=(
            "boreas-2024-02-29-14-39"
            "boreas-2024-02-29-14-53"
            "boreas-2024-02-29-15-02"
            "boreas-2024-02-29-15-11"
        )
    elif [[ $route == "skyway" ]]; then
        SEQUENCES=(
            "boreas-2024-02-29-11-54"
            "boreas-2024-02-29-12-13"
            "boreas-2024-02-29-12-31"
            "boreas-2024-02-29-12-48"
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