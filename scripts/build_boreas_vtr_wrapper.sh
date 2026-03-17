# Assumes that ROOTDIR is set and pointing to root directory
# Need to additionally set VTRSRC variable
export VTRSRC=$ROOTDIR/external/vtr3
source /opt/ros/humble/setup.bash
source ${VTRSRC}/main/install/setup.bash # source the vtr3 environment
cd $ROOTDIR/src # go to the root directory
MAKEFLAGS="-j$(($(nproc --all) / 4 + 1))" colcon build --parallel-workers 1 --packages-select vtr_testing_common vtr_testing_lidar vtr_testing_radar vtr_testing_radar_lidar vtr_testing_aeva --packages-ignore $(colcon list | grep -v -E 'vtr_testing_common|vtr_testing_aeva|vtr_testing_lidar|vtr_testing_radar|vtr_testing_radar_lidar' | awk '{print $1}' | tr '\n' ' ') --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release


cd $ROOTDIR