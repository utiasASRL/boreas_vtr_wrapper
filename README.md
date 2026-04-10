## Download boreas_vtr_wrapper

This package contains testing code for lidar and radar pipeline. Download it do your local filesystem.

```Bash
git clone git@github.com:utiasASRL/boreas_vtr_wrapper.git
cd boreas_vtr_wrapper
git checkout script_update
git submodule update --init --recursive
```

## Build boreas_vtr_wrapper Docker Image

This builds a image that has all dependencies installed.

```Bash
bash scripts/build_docker.sh
```

## Start the boreas_vtr_wrapper Docker container

This will either attach to a boreas_vtr_wrapper container that's already running or create a new container which gets deleted upon exit.

```Bash
bash scripts/run_docker.sh
```

FYI: This will always run `entrypoint.sh` which tries to source the python venv (see below). Also, the container expects Boreas sequences to be stored at `$VTRRDATA`. If your data is not stored here, then consider mounting a volume in the shell script like `-v host_path:container_path:ro`. In `setup_container.sh`, point `$VTRRESULT` and `$VTRRDATA` to where you want to store results and data.

## Build and Install VT&R3 + boreas_vtr_wrapper

Inside the container

```Bash
bash scripts/build_packages.sh
```

If this build is crashing your computer, consider changing `scripts/build_vtr3.sh` and `scripts/build_boreas_vtr_wrapper.sh` to `MAKEFLAGS="-j1"`

## Create a python venv to install pyboreas

Within the running container

```Bash
bash scripts/create_venv.sh
```

# Running Experiments

## Visualization

### RVIZ
First launch RVIZ for visualization:

```Bash
source /opt/ros/humble/setup.bash               # source the ROS environment
ros2 run rviz2 rviz2 -d ${VTRSRC}/rviz/radar.rviz # launch rviz
```

Then in another terminal, launch `rqt_reconfigure` for control. Currently supported dynamic reconfigure parameters: `control_test.play` and `control_test.delay_millisec`

```Bash
source /opt/ros/humble/setup.bash
ros2 run rqt_reconfigure rqt_reconfigure
```

### Foxglove
An alternative visualization approach is to use [Foxglove](https://foxglove.dev). This approach has the advantage of being able to locally visualize ROS topics even in cases where the code is running on a remote machine. For convinience, the Foxglove WebSocket is already installed as part of the standard Dockerfile. This allows you to connect to the remote machine using the web browser or by downloading the [Foxglove Studio](https://foxglove.dev/download), as long as your local machine can reach the remote machine in some manner. 

To use the WebSocket, open another terminal window inside of a set up Docker container and run
```Bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Then, either [in the browser](https://studio.foxglove.dev) or in the Foxglove Studio application, navigate to `Open Connection -> Foxglove WebSocket` and enter `ws://REMOTE_IP:8765`, where `REMOTE_IP` is the ping-able IP address of your remote machine. Afterwards, all remote machine ROS topics should be visualizable using the Foxglove interface (once a test is running). The browser will sometimes take a long time to load, so the application is generally recommended. Additional information about using Foxglove WebSocket can be found at https://github.com/foxglove/ros-foxglove-bridge/.

## Odometry (Teach) and Localization (Repeat)

The general form of running and evaluating a test inside the Docker container is
```Bash
bash runtime/run_test.sh ${MODE} ${SENSOR} ${SEQUENCES}
bash runtime/run_eval.sh ${MODE} ${SENSOR} ${SEQUENCES}
```
where `MODE = [odometry, localization]`, `SENSOR = [radar, lidar, radar_lidar]`, and `SEQUENCES` is either one (for odometry) or two (for localization) Boreas sequence names. Consider the examples below for radar.

Consider, as an example, the following sequences for odometry and localization.
```Bash
# Choose a Teach (ODO_INPUT) and Repeat (LOC_INPUT) run from boreas dataset
ODO_INPUT=boreas-2020-11-26-13-58
LOC_INPUT=boreas-2021-01-26-10-59
```
Note, it is not required to define these variables, as you can input the sequence name as an argument directly. If it is desired to do a localization test, it is first required that an odometry result is generated for the sequence against which a localization attempt is desired.

Run and evaluate a single radar odometry test using
```Bash
bash runtime/run_test.sh odometry radar ${ODO_INPUT}
bash runtime/run_eval.sh odometry radar ${ODO_INPUT}
```

Run and evaluate a single radar localization test using
```Bash
bash runtime/run_test.sh localization radar ${ODO_INPUT} ${LOC_INPUT}
bash runtime/run_eval.sh localization radar ${ODO_INPUT}
```

Note, that the evaluation scripts both only take in an odometry sequence. This is because the output of a localization run against a map constructed from an odometry sequence is stored under the odometry sequence result subfolder. The evaluation script evaluates all localization sequences contained within the odometry sequence subfolder at the same time. `runtime/config` contains the config files corresponding to the `SENSOR` used in Teach and Repeat. Note that for radar localization against lidar submaps, use lidar for the Teach phase and radar_lidar for the Repeat phase. A copy of the `.yaml` config file will also be saved in `results` for each pipeline run. When debugging, increase threads in the `.yaml` file for faster pipeline runs (will result in slight changes in results) but use single thread for best results.

## Running Experiments in Parallel

Assuming you want to run odometry or localization for multiple test sequences in parallel, it is possible to do so by running

The general form of running and evaluating tests on multiple sequences in parallel is
```Bash
bash runtime/run_parallel_test.sh ${MODE} ${SENSOR}
```
where `MODE = [odometry, localization]`, `SENSOR = [radar, lidar, radar_lidar]`. This script runs all tests, either odometry or localization, and evaluates them afterwards. Note, SEQUENCES are not provided as an input for this script, as the specific list of sequences desired to be tested in parallel must be set inside of the script file. Consider the examples below for radar localization.

```Bash
bash runtime/run_parallel_test.sh localization radar
```
Note, running this script assumes that the REFERENCE sequence, set inside of run_parallel_test.sh, has an already completed odometry test.

You can monitor the progress of each test by going to the log file of each test.

The log file should be located at

`${VTRRESULT}/${SENSOR}/${ODO_INPUT}/${ODO_INPUT}/<some name based on time>.log`

for odometry and at

`${VTRRESULT}/${SENSOR}/${ODO_INPUT}/${LOC_INPUT}/<some name based on time>.log`

for localization, where `${VTRRESULT}` is set in `setup_container.sh`. After the evaluation of the tests is complete, you should see the output in the terminal. Various other results can be found in the `${VTRRESULT}` directory.

## Plotting Submaps

`external/vtr3_python` contains a script `plot_submaps.py` which can be used to plot the lidar or radar submaps from Teach pass. 

## [License](./LICENSE)
