#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

python3 -m localization_dfo.pipeline_dfo \
    --experiment-name 1-suburb-industrial-farm-prior-DRO-lidar-odom \
    --map-sequence boreas-2024-12-03-12-54 --map-odometry odom \
    --loc-sequence boreas-2025-01-08-11-22 boreas-2025-01-08-12-28 \
    --radar-start-frame 0 --imfil-budget 84 \
    --save-labels --save-predictions --overwrite --device cuda:1

python3 -m localization_dfo.pipeline_dfo \
    --experiment-name 1-suburb-industrial-farm-prior-DRO-lidar-odom \
    --map-sequence boreas-2024-12-05-14-12 --map-odometry odom \
    --loc-sequence boreas-2024-12-23-16-27 boreas-2024-12-23-16-44 boreas-2024-12-23-17-01 \
    --radar-start-frame 0 --imfil-budget 84 \
    --save-labels --save-predictions --overwrite --device cuda:1
