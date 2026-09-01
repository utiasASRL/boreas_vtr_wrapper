#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

python3 -m localization_dfo.pipeline_dfo \
    --experiment-name 1-suburb-industrial-farm-prior-DRO-lidar-odom \
    --map-sequence boreas-2024-12-03-13-13 --map-odometry odom \
    --loc-sequence boreas-2024-12-10-12-07 \
    --radar-start-frame 2626 --imfil-budget 84 \
    --save-labels --save-predictions --append --device cuda:3

python3 -m localization_dfo.pipeline_dfo \
    --experiment-name 1-suburb-industrial-farm-prior-DRO-lidar-odom \
    --map-sequence boreas-2024-12-03-13-13 --map-odometry odom \
    --loc-sequence boreas-2024-12-10-12-38 \
    --radar-start-frame 0 --imfil-budget 84 \
    --save-labels --save-predictions --overwrite --device cuda:3
