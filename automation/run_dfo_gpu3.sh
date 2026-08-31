#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

# Continuation queue: run after the active DRO-submap localization finishes.
python3 -m localization_dfo.pipeline_dfo \
    --experiment-name 1-suburb-industrial-farm-prior-DRO-lidar-odom \
    --map-sequence boreas-2025-07-18-14-55 --map-odometry odom \
    --loc-sequence boreas-2025-07-18-15-12 boreas-2025-07-18-15-30 boreas-2025-07-18-15-48 \
    --radar-start-frame 0 \
    --save-labels --save-init --save-predictions --overwrite --device cuda:3

python3 -m localization_dfo.pipeline_dfo \
    --experiment-name 1-suburb-industrial-farm-prior-DRO-lidar-odom \
    --map-sequence boreas-2024-12-03-13-13 --map-odometry odom \
    --loc-sequence boreas-2024-12-10-12-07 boreas-2024-12-10-12-38 \
    --radar-start-frame 0 --imfil-budget 84 \
    --save-labels --save-predictions --overwrite --device cuda:3
