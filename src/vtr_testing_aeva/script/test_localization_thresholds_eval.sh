## this script assumes the following environment variables are set:
##   VTRRROOT VTRRDATA VTRRRESULT

# Get arguments
ODO_INPUT=$1

# Set results subfolder, VTRRESULT is set in setup_container.sh
export VTRRRESULT=${VTRRESULT}/aeva
mkdir -p ${VTRRRESULT}

# Log
echo "Evaluating localization to reference sequence ${ODO_INPUT}, storing result to ${VTRRRESULT}/${ODO_INPUT}"

# Source the VTR environment with the testing package
source ${VTRRROOT}/src/install/setup.bash
source ${VTRROOT}/venv/bin/activate

result_dir="${VTRRRESULT}/${ODO_INPUT}"
# loc_inputs=($(ls ${result_dir} | grep -v ${ODO_INPUT}))
loc_inputs=($(find ${result_dir} -type d -name '*threshold*' -not -name '*logs*' -exec basename {} \;))

echo "Localization tests found: ${loc_inputs[@]}"

for LOC_INPUT in "${loc_inputs[@]}"; do
    # Dump localization result to boreas expected format (txt file)
    python ${VTRRROOT}/src/vtr_testing_aeva/script/boreas_generate_localization_result.py --dataset ${VTRRDATA} --path ${VTRRRESULT}/${ODO_INPUT} --input_loc_dir ${LOC_INPUT}

    # Evaluate the result using the evaluation script
    python -m pyboreas.eval.localization_aeva --gt ${VTRRDATA} --pred ${VTRRRESULT}/${ODO_INPUT}/localization_result --ref_seq ${ODO_INPUT} --ref_sensor aeva --test_sensor aeva --dim 3  --plot ${VTRRRESULT}/${ODO_INPUT}/localization_result/lidar-lidar --loc_dir ${LOC_INPUT}

    # Copy all .log files to the new folder
    mkdir -p ${VTRRRESULT}/${ODO_INPUT}/localization_result/${LOC_INPUT}_logs
    cp ${VTRRRESULT}/${ODO_INPUT}/${LOC_INPUT}/*.log ${VTRRRESULT}/${ODO_INPUT}/localization_result/${LOC_INPUT}_logs/
done




