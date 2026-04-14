# Set ROOTDIR to the root directory of the project
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
export ROOTDIR=$(dirname "$SCRIPT_DIR")
cd $ROOTDIR
docker build -t boreas_wrapper_$(whoami) -f Dockerfile \
    --build-arg USERID=$(id -u) \
    --build-arg GROUPID=$(id -g) \
    --build-arg USERNAME=$(whoami) \
    --build-arg HOMEDIR=${HOME} .
cd $ROOTDIR