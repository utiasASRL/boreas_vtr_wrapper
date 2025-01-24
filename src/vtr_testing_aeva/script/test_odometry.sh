## this script assumes the following environment variables are set:
##   VTRRROOT VTRRDATA VTRRRESULT
## example usage: test_odometry.sh boreas-2021-09-02-11-42

# Get arguments
ODO_INPUT=$1
if [ $# -eq 2 ]; then
  PARAM_FILE=$2
else
  PARAM_FILE=${VTRRROOT}/src/vtr_testing_aeva/config/aeva.yaml
fi

# Log
echo "Running odometry on sequence ${ODO_INPUT}, storing result to ${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT}"

# Source the VTR environment with the testing package
source ${VTRRROOT}/install/setup.bash

# Copy over parameter file
cp ${PARAM_FILE} ${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT}/aeva_odometry_config.yaml

ros2 run vtr_testing_aeva vtr_testing_aeva_aeva_odometry \
  --ros-args -p use_sim_time:=true \
  -r __ns:=/vtr \
  --params-file ${PARAM_FILE} \
  -p data_dir:=${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT} \
  -p odo_dir:=${VTRRDATA}/${ODO_INPUT}
