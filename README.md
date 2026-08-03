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

FYI: This will always run `entrypoint.sh` which tries to source the python venv (see below). Also, the container expects Boreas sequences to be stored at `$VTRRDATA`. If your data is not stored here, then consider mounting a volume in the shell script like `-v host_path:container_path:ro`

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

## Visualization (Work in Progress)

First launch RVIZ for visualization:

```Bash
source /opt/ros/galactic/setup.bash               # source the ROS environment
ros2 run rviz2 rviz2 -d ${VTRSRC}/rviz/radar.rviz # launch rviz
```

Then in another terminal, launch `rqt_reconfigure` for control. Currently supported dynamic reconfigure parameters: `control_test.play` and `control_test.delay_millisec`

```Bash
source /opt/ros/galactic/setup.bash
ros2 run rqt_reconfigure rqt_reconfigure
```

## Odometry (Teach) and Localization (Repeat)

```Bash
export VTRRROOT=${VTRROOT}/boreas_vtr_wrapper # location of this repository CHANGE THIS!
export VTRRDATA=${VTRDATA}/boreas/sequences  # dataset location (where the boreas-xxxxx folders at) CHANGE THIS!
export VTRRRESULT=${VTRTEMP}/radar/boreas    # default result location
mkdir -p ${VTRRRESULT}
```

```Bash
source ${VTRRROOT}/src/install/setup.bash
# Choose a Teach (ODO_INPUT) and Repeat (LOC_INPUT) run from boreas dataset
ODO_INPUT=boreas-2020-11-26-13-58
LOC_INPUT=boreas-2021-01-26-10-59
```

Odometry:
```Bash
bash ${VTRRROOT}/src/vtr_testing_radar/script/test_odometry.sh ${ODO_INPUT}
bash ${VTRRROOT}/src/vtr_testing_radar/script/test_odometry_eval.sh ${ODO_INPUT}
```

Localization:
```Bash
bash ${VTRRROOT}/src/vtr_testing_radar/script/test_localization.sh ${ODO_INPUT} ${LOC_INPUT}
# Evaluation:
bash ${VTRRROOT}/src/vtr_testing_radar/script/test_localization_eval.sh ${ODO_INPUT}
```


## Running Experiments in Parallel

Assuming you want to run odometry or localization for all test sequences in parallel.

Inside the `script` folder of all three testing packages (`vtr_testing_<...>`), you can find the following two bash script:

- `parallel_test_odometry.sh`
- `parallel_test_localization.sh`


All you need to do is run one of the above bash scripts **inside the contaner**. 

```
bash <path to parallel_test_odometry.sh or parallel_test_localization.sh>
```

For example,

```
bash ${VTRRROOT}/src/vtr_testing_radar/script/parallel_test_localization.sh
```

Then monitor progress by going to the log file of each test.

The log file should be located at

`~/ASRL/temp/[radar, lidar, radar_lidar]/boreas/<boreas-2020-11-26-13-58>/<boreas-2020-11-26-13-58>/<some name based on time>.log`

Understand what these scripts do:

Using `parallel_test_odometry.sh` from `src/vtr_testing_radar/script` as an example, the script does the following:

1. Define sequences we need to run for odometry

```
# odometry sequences
SEQUENCES=(
  'boreas-2020-11-26-13-58'  # Note this is the localization reference run, you must run this in order to run localization tests
  'boreas-2020-12-04-14-00'
  'boreas-2021-01-26-10-59'
  'boreas-2021-02-09-12-55'
  'boreas-2021-03-09-14-23'
  'boreas-2021-04-22-15-00'
  'boreas-2021-06-29-18-53'
  'boreas-2021-06-29-20-43'
  'boreas-2021-09-08-21-00'
)
```

2. Set max number of sequences to run in parallel

```
# maximum number of jobs running in parallel
GROUPSIZE=20
```

3. Setup up directories

These directories are defined using the environment variables in `Setup VTR3 Directories` section.

I suggest you don't change them.

For `VTRRDATA`, it is supposed to be the directory that contains all boreas sequences (i.e. `boreas-....`). You can create a symlink from boreas dataset on /nas to this directory.

```
# define the following environment variables VTRR=VTR RaDAR
export VTRRROOT=${VTRROOT}/boreas_vtr_wrapper # location of this repository CHANGE THIS!
export VTRRDATA=${VTRDATA}/boreas/sequences  # dataset location (where the boreas-xxxxx folders at) CHANGE THIS!
export VTRRRESULT=${VTRTEMP}/radar/boreas    # result location MAYBE CHANGE THIS!
mkdir -p ${VTRRRESULT}
```

4. Define path to test scripts

```
ODOMETRY_SCRIPT="${VTRRROOT}/src/vtr_testing_radar/script/test_odometry.sh"
ODOMETRY_EVAL_SCRIPT="${VTRRROOT}/src/vtr_testing_radar/script/test_odometry_eval.sh"
```

These are bash scripts that will run odometry test (using `ros2 run ...`) and evaluation.

5. Run odometry tests in parallel

The following code runs at most `GROUPSIZE` odometry tests in parallel by calling the `$ODOMETRY_SCRIPT` test script with each of the sequence specified in `SEQUENCES`.

```
declare -A pids
for seq in ${SEQUENCES[@]}; do
  echo "Executing command: bash $ODOMETRY_SCRIPT $seq &>/dev/null &"
  ### command to execute
  bash $ODOMETRY_SCRIPT $seq &>/dev/null &
  ###
  pids[${seq}]=$!
  # wait for all pids to finish if reached group size
  if [[ ${#pids[@]} -ge ${GROUPSIZE} ]]; then
    for key in ${!pids[@]}; do
      wait ${pids[${key}]}
      echo "Process ${key} finished with return code ${?}"
      unset pids[${key}]
    done
  fi
done
for key in ${!pids[@]}; do
  wait ${pids[${key}]}
  echo "Process ${key} finished with return code ${?}"
  unset pids[${key}]
done
```

6. Run evaluation

When all sequences are finished, the following code runs pyboreas odometry evaluation on the result of each sequence. You should see output in terminal.

```
for seq in ${SEQUENCES[@]}; do
  echo "Executing command: bash $ODOMETRY_EVAL_SCRIPT $seq"
  bash $ODOMETRY_EVAL_SCRIPT $seq
done
```

## To-do
- Correctly load initial velocity conditions when starting from start frame != 0
- Add start frame control to localization
- Add submap saving to all pipelines

## [License](./LICENSE)
