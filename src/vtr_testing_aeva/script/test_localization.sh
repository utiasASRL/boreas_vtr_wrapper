## this script assumes the following environment variables are set:
##   VTRRROOT VTRRDATA VTRRRESULT
## example usage: test_localization.sh boreas-2021-09-02-11-42 boreas-2021-09-07-09-35

# Get arguments
ODO_INPUT=$1
LOC_INPUT=$2

# Using the config within the aeva package
PARAM_FILE=${VTRRROOT}/src/vtr_testing_aeva/config/aeva_boreas.yaml
  
# Save param file
SAVE_CONFIG=aeva_localization_config.yaml
mkdir -p ${VTRRRESULT}/${ODO_INPUT}
cp ${PARAM_FILE} ${VTRRRESULT}/${ODO_INPUT}/${SAVE_CONFIG}
echo "PARAM FILE IS ${PARAM_FILE}"

# Log
echo "Running localization on sequence ${LOC_INPUT} to reference sequence ${ODO_INPUT}, storing result to ${VTRRRESULT}/${ODO_INPUT}/${LOC_INPUT}"

# Source the VTR environment with the testing package
source ${VTRRROOT}/src/install/setup.bash

graph_dir=${VTRRRESULT}/${ODO_INPUT}/${LOC_INPUT}/graph
if [ -d $graph_dir ]; then
  # Count the number of directories inside "graph"
  dir_count=$(ls -l $graph_dir | grep -c ^d)
  
  if [ $dir_count -gt 1 ]; then
    read -p "The directory $graph_dir is not empty and contains $dir_count other directories. Do you want to delete it and create an empty one? (yes/no) " response
    if [ "$response" == "no" ]; then
      exit
    fi
  fi
fi

rm -r ${VTRRRESULT}/${ODO_INPUT}/${LOC_INPUT}
mkdir -p ${VTRRRESULT}/${ODO_INPUT}/${LOC_INPUT}
cp -r ${VTRRRESULT}/${ODO_INPUT}/${ODO_INPUT}/*  ${VTRRRESULT}/${ODO_INPUT}/${LOC_INPUT}
ros2 run vtr_testing_aeva vtr_testing_aeva_aeva_boreas_localization  \
  --ros-args -p use_sim_time:=true \
  -r __ns:=/vtr \
  --params-file ${PARAM_FILE} \
  -p data_dir:=${VTRRRESULT}/${ODO_INPUT}/${LOC_INPUT} \
  -p odo_dir:=${VTRRDATA}/${ODO_INPUT} \
  -p loc_dir:=${VTRRDATA}/${LOC_INPUT}