#!/bin/bash

# Define file paths manually
seq_list=(
  "boreas-2024-01-09-14-00"
  "boreas-2024-01-23-11-45"
  "boreas-2024-01-23-12-15"
  "boreas-2024-01-23-12-32"
  "boreas-2024-01-25-11-44"
)

for seq in "${seq_list[@]}"; do
    echo "Processing sequence: $seq"
    python vel_estimation_horde.py --seq $seq
done