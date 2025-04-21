# Set up VTR directory pointers
export VTRROOT=$ROOTDIR                     # This is required for some internal scripts
export VTRRROOT=$ROOTDIR
export VTRSRC=$ROOTDIR/external/vtr3
export VTRRESULT=$ROOTDIR/results           # POINT THIS TO WHERE YOU WANT TO STORE RESULTS
# export VTRRDATA=$ROOTDIR/data               # POINT THIS TO DATA DIRECTORY
export VTRRDATA=/home/katya/Documents/Data/aevaii 

# Source setups
source /opt/ros/humble/setup.bash
source $VTRSRC/main/install/setup.bash # source the vtr3 environment
source $VTRRROOT/src/install/setup.bash

# Create directories if they don't exist
mkdir -p $VTRRESULT
mkdir -p $VTRRDATA

# Activate venv
source $ROOTDIR/venv/bin/activate

# export OMP_NUM_THREADS=6   # used to control the number of threads the container can use
# export NEPTUNE_API_TOKEN=temp # Input your Neptune API here