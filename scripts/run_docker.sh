# Set ROOTDIR to the root directory of the project
# KTODO: ADD GPU BACK TO DOCKER
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
export ROOTDIR=$(dirname "$SCRIPT_DIR")

container_state=$(docker inspect -f '{{.State.Running}}' boreas_wrapper_$(whoami) 2>/dev/null)

if [ "$container_state" = "true" ]
then
	echo 'Container already running, joining it now.'
	docker exec -it boreas_wrapper_$(whoami) /entrypoint.sh
else
	echo 'New container run initialized.'
	docker run -it --rm --name boreas_wrapper_$(whoami) \
	--gpus all \
	--privileged \
	--network=host \
	--ipc=host \
	-e DISPLAY=$DISPLAY \
	-e ROOTDIR=$ROOTDIR \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v ${HOME}/.Xauthority:${HOME}/.Xauthority:rw \
	-v $ROOTDIR:$ROOTDIR:rw \
	-v "/media/asrl/Extreme SSD/ASRL/boreas/data":/boreas_data:rw \
	-v "/media/asrl/Extreme SSD/ASRL/boreas/boreas_vtr_wrapper_results":"/media/asrl/Extreme SSD/ASRL/boreas/boreas_vtr_wrapper_results":rw \
	-w $ROOTDIR boreas_wrapper_$(whoami)
fi
cd $ROOTDIR
