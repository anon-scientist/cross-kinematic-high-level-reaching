# Kinematic High-Level Reaching Benchmark (HLR-K), part of the CRoSS benchmark suite
This repository contains the kinematic variant of the High-Level Reaching environment from the CRoSS suite.
In this version, robot motion is computed analytically using forward and/or inverse kinematics, without requiring the Gazebo simulator.
The kinematic setup preserves the same observation, action, and reward structure as the physically simulated environment, but runs significantly faster, making it suitable for rapid prototyping, algorithm development, and hyperparameter tuning.
The full simulated version and additional benchmarks are linked from the main [CRoSS](https://github.com/anon-scientist/continual-robotic-simulation-suite/) repository.

*This repository is provided exclusively for anonymous review purposes.
To preserve anonymity during the submission process, the repository is maintained in read-only mode and does not accept issues, pull requests, or discussions at this stage.
Upon acceptance of the paper, we will make the full public version of this project available under our official organization account.*

## Overview
The High-Level Reaching (HLR) benchmark is a continual reinforcement learning environment in which a simulated 7-DoF robotic arm must move its end-effector to task-specific 3D goal positions.
Control is performed at the Cartesian level: instead of commanding joint angles, the agent issues discrete movement commands that shift the end-effector in small steps along the X, Y, or Z axes.
This allows the robot to be controlled in a simple, abstract manner while still requiring precise spatial reasoning to reach the target.

The kinematic version provided in this repository replaces physics-based simulation with analytical forward and inverse kinematics.
The robot’s movement is computed directly from kinematic equations, preserving the exact observation, action, and reward structure of the simulated version while running significantly faster.

The benchmark consists of ten tasks, modeled after the [Continual World](https://github.com/awarelab/continual_world) task suite:
hammer, push_wall, faucet_close, push_back, stick_pull,
handle_press_side, push, shelf_place, window_close, and peg_unplug_side.
Each task defines a different 3D goal position and requires the agent to move the end-effector efficiently and accurately using only discrete Cartesian motions.

## Requirements
-   icrl repository
-   cl_experiment repository
-   Apptainer software
-   icrl/icrl.def for building the container

## How to Run
1. Clone this repository in a folder, where all other required repositories will be cloned into!!!
```bash
git clone https://github.com/anon-scientist/cross-kinematic-high-level-reaching
```
2. Clone both the icrl and cl_experiment repositories from the requirements above.
(Run these where you want them to end up in your file system)
``` bash
git clone https://github.com/anon-scientist/icrl
git clone https://github.com/anon-scientist/cl_experiment
```
3. Create the Apptainer container from the `icrl.def` file in the icrl repository.
```bash
cd icrl
apptainer build ../my_container.sif icrl.def
cd ..
```
4. (Optional) Modify the cmd line parameters in `main.bash` of this repository to what you want to test.
5. Run the `main.bash` by executing it with the apptainer image.
    * change "path_to_benchmark_repo" to the path where you cloned the repository into (including its name)
    * Make sure that the image has execute permission for the `main.bash`
```bash
apptainer exec my_container.sif path_to_benchmark_repo/main.bash
```
6. Look at the simulation results located in a new results folder inside the cloned repository

## Files


## Command Line Parameters
Here are some of the relevant hyperparameters the RL-Framework accepts. For more information look at the `main.bash` or the individual relevant python files.
```bash
--benchmark # The name for the benchmark.
--exp_id # Name of the experiment to use as an identifier for the generated result.
--root_dir # Directory where all experiment results and logs are stored.
--training_duration # Defines the number of iterations á training_duration_unit.
--evaluation_duration # Defines the number of iterations á evaluation_duration_unit.
--training_duration_unit # Sets the unit (abstraction level) t
--evaluation_duration_unit # Sets the unit (abstraction level) t
--max_steps_per_episode # Sets the number of steps after which an episode gets terminated.
--task_list # tasks to execute and in what order
--start_task # set the task from where to start the experiments
--eval_start_task # at what point evaluation should start being performed
--exploration_start_task # all other tasks before this are pure babbeling (random moves)
--training_duration_task_0 # first task duration (for babbeling phases)
--train_batch_size # Defines the mini-batch size that is used for training.
--algorithm # Defines the algorithm to use for the training
--goal_discrepency_threshold # max distance to goal that is considered reached
--action_speed # step-size for each cartesian action
--reward_skew # To give the reward non-linearity
--exploration # The exploration strategy the agent should use.
--replay_buffer # The type of buffer to use
--capacity # the replay buffer capacity
```

## Related Repositories
* [**CRoSS** - Entry Repository](https://github.com/anon-scientist/continual-robotic-simulation-suite/)
* [ICRL - RL-Framework Repository](https://github.com/anon-scientist/icrl/)
* [CL_Experiments - Utils Repository](https://github.com/anon-scientist/cl_experiments/)
* [HLR with Physics Simulation](https://github.com/anon-scientist/cross-high-level-reaching)
