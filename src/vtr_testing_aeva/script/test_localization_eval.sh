## this script assumes the following environment variables are set:
##   VTRRROOT VTRRDATA VTRRRESULT
## example usage: test_localization.sh boreas-2021-09-02-11-42 boreas-2021-09-07-09-35

# Get arguments
ODO_INPUT=$1

# Log
echo "Evaluating localization to reference sequence ${ODO_INPUT}, storing result to ${VTRRRESULT}/${ODO_INPUT}"

# Source the VTR environment with the testing package
source ${VTRRROOT}/src/install/setup.bash
source ${VTRROOT}/venv/bin/activate

# Dump localization result to boreas expected format (txt file)
python ${VTRRROOT}/src/vtr_testing_aeva/script/boreas_generate_localization_result.py --dataset ${VTRRDATA} --path ${VTRRRESULT}/${ODO_INPUT}

# Evaluate the result using the evaluation script
python -m pyboreas.eval.localization_aeva --gt ${VTRRDATA} --pred ${VTRRRESULT}/${ODO_INPUT}/localization_result --ref_seq ${ODO_INPUT} --ref_sensor aeva --test_sensor aeva --dim 3  --plot ${VTRRRESULT}/${ODO_INPUT}/localization_result/lidar-lidar

# Copy all .log files to the results folder
for folder in ${VTRRRESULT}/${ODO_INPUT}/boreas*/; do
    if [[ "$(basename "$folder")" != "${ODO_INPUT}" ]]; then
        cd "$folder" || continue
        mkdir -p ${VTRRRESULT}/${ODO_INPUT}/localization_result/$(basename "$folder")_logs
        cp *.log ${VTRRRESULT}/${ODO_INPUT}/localization_result/$(basename "$folder")_logs/
    fi
done