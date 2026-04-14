# Assumes that ROOTDIR is set and pointing to mm_masking root directory
cd $ROOTDIR
virtualenv venv
touch venv/COLCON_IGNORE
source venv/bin/activate

pip install -r $ROOTDIR/requirements.txt
deactivate

cd $ROOTDIR
