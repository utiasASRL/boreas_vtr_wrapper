#!/bin/bash

# Define commands
commands=(
    "bash runtime/run_test.sh localization aeva 2025-01-08-a-glen 2024-12-03-b-glen"
    "sleep 2"
    "bash runtime/run_test.sh localization aeva 2024-12-10-b-hwy7-fwd 2024-12-03-hwy7-fwd"
    "sleep 2"
    "bash runtime/run_test.sh localization aeva 2024-12-10-b-hwy7-fwd 2024-12-10-a-hwy7-fwd"
    "sleep 2"
    "bash runtime/run_test.sh localization aeva 2024-12-23-new2 2024-12-23-new5"
    "sleep 2"
    "bash runtime/run_test.sh localization aeva 2024-12-23-new2 2024-12-23-new4"
)

# Loop and execute
for cmd in "${commands[@]}"; do
    echo "Executing: $cmd"
    eval $cmd
    if [ $? -ne 0 ]; then
        echo "Command failed: $cmd"
        exit 1
    fi
done

echo "All commands executed."