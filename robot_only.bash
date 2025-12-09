#!/bin/bash
# starts a Python script that demonstrates use of the robot, without environment or RL framework

SRC_PATH=${1}
PROJECT_DIR="cross-kinematic-high-level-reaching"
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
echo "PYTHONPATH: ${PYTHONPATH}"
# *--------------------------------------------------------------------

python3 robot_only.py
