# Assumes that ROOTDIR is set and pointing to mm_masking root directory
cd $ROOTDIR
virtualenv venv
touch venv/COLCON_IGNORE
source venv/bin/activate

pip install -r $ROOTDIR/requirements.txt
pip install -e "$ROOTDIR/external/vtr3_pose_graph"
bash "$ROOTDIR/scripts/install_nksr.sh"

deactivate

cd $ROOTDIR
