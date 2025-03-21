# localization sequences
SEQUENCES=(
  'boreas-2020-12-04-14-00'
  'boreas-2021-01-26-10-59'
  'boreas-2021-02-09-12-55'
  'boreas-2021-03-09-14-23'
  'boreas-2021-06-29-18-53'
  'boreas-2021-09-08-21-00'
  'boreas-2021-10-05-15-35'
  'boreas-2021-10-26-12-35'
  'boreas-2021-11-06-18-55'
  'boreas-2021-11-28-09-18'
)

# sequences which use the "new" navtech radar:
NEW_RADAR_SEQUENCES=(
  'boreas-2021-10-05-15-35'
  'boreas-2021-10-26-12-35'
  'boreas-2021-11-06-18-55'
  'boreas-2021-11-28-09-18'
)

REFERENCE='boreas-2020-11-26-13-58' # 'boreas-2021-09-02-11-42'

# maximum number of jobs running in parallel
GROUPSIZE=3

# define the following environment variables VTRR=VTR RaDAR
export VTRRROOT=${VTRROOT}/boreas_vtr_wrapper # location of this repository CHANGE THIS!
# export VTRRDATA=${VTRDATA}/boreas/sequences  # dataset location (where the boreas-xxxxx folders at) CHANGE THIS!
export VTRRRESULT=${VTRTEMP}/radar_lidar    # result location MAYBE CHANGE THIS!
mkdir -p ${VTRRRESULT}

LOCALIZATION_SCRIPT="${VTRRROOT}/src/vtr_testing_radar_lidar/script/test_localization.sh"
LOCALIZATION_EVAL_SCRIPT="${VTRRROOT}/src/vtr_testing_radar_lidar/script/test_localization_eval.sh"

declare -A pids

for seq in ${SEQUENCES[@]}; do
  PARAM_FILE=${VTRRROOT}/src/vtr_testing_radar_lidar/config/boreas.yaml
  for item in $NEW_RADAR_SEQUENCES; do
    if [ "$seq" == "$item" ]; then
      PARAM_FILE=${VTRRROOT}/src/vtr_testing_radar_lidar/config/boreas_new_radar.yaml
    fi
  done
  echo "Executing command: bash $LOCALIZATION_SCRIPT $REFERENCE $seq $PARAM_FILE &>/dev/null &"
  ### command to execute
  bash $LOCALIZATION_SCRIPT $REFERENCE $seq $PARAM_FILE &>/dev/null &
  ###
  pids[${seq}]=$!
  # wait for all pids to finish if reached group size
  if [[ ${#pids[@]} -ge ${GROUPSIZE} ]]; then
    for key in ${!pids[@]}; do
      wait ${pids[${key}]}
      echo "Process ${key} finished with return code ${?}"
      unset pids[${key}]
    done
  fi
done
for key in ${!pids[@]}; do
  wait ${pids[${key}]}
  echo "Process ${key} finished with return code ${?}"
  unset pids[${key}]
done

echo "Executing command: bash $LOCALIZATION_EVAL_SCRIPT $REFERENCE"
bash $LOCALIZATION_EVAL_SCRIPT $REFERENCE
