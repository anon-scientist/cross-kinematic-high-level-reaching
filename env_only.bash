#!/bin/bash
# starts a Python script that demonstrates use of the robot, without environment or RL framework

SRC_PATH=${1}
PROJECT_DIR="robot-arm-no-gazebo-scenario"
ROOT_PATH="${SRC_PATH}/${PROJECT_DIR}"

if [ -d "$ROOT_PATH" ]; then
    echo "Root path ${ROOT_PATH} confirmed!"
else
    echo "Root path ${ROOT_PATH} does not exsist. Please use > main.bash [path_to_your_git_folder] to set the correct directory!"
    exit 1
fi

# PYTHONPATH - PYTHONPATH - PYTHONPATH --------------------------------
export PYTHONPATH=$PYTHONPATH:${ROOT_PATH}/src
export PYTHONPATH=$PYTHONPATH:${SRC_PATH}/icrl/src
export PYTHONPATH=$PYTHONPATH:${SRC_PATH}/dcgmm/src
export PYTHONPATH=$PYTHONPATH:${SRC_PATH}/cl_experiment/src
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
# *--------------------------------------------------------------------

python3 env_only.py