## this script assumes the following environment variables are set:
##   VTRRROOT VTRRDATA VTRRRESULT
## example usage: test_preprocessing.sh boreas-2021-09-02-11-42

# Get arguments
ODO_INPUT=$1
if [ $# -eq 2 ]; then
  PARAM_FILE=$2
else
  PARAM_FILE=${VTRRROOT}/src/vtr_testing_lidar/config/boreas.yaml
fi

# Log
echo "Running preprocessing on sequence ${ODO_INPUT}, storing result to ${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT}"

# Source the VTR environment with the testing package
source ${VTRRROOT}/install/setup.bash

# Copy over parameter file
cp ${PARAM_FILE} ${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT}/lidar_preprocessing_config.yaml

ros2 run vtr_testing_lidar vtr_testing_lidar_boreas_preprocessing \
  --ros-args -p use_sim_time:=true \
  -r __ns:=/vtr \
  --params-file ${PARAM_FILE} \
  -p data_dir:=${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT} \
  -p odo_dir:=${VTRRDATA}/${ODO_INPUT}
