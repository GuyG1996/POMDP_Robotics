# Robotics Decision Making Under Uncertainty

## Problem Description

Every afternoon, a mother returns from work, picks up her baby from daycare, and comes home. She activates her home assistant robot to entertain her baby while she rests. The robot's basic skills include navigating to a location, picking a toy, and placing it in the baby's lap. However, these actions are not always successful due to the uncertainty of the environment.

## How to Run

1. Extract and place `my_control.py` and `my_experiment.py` in the `task3_env/scripts` directory.
2. Extract and place `CMakeLists.txt` file and replace yours inside the `task3_env` directory.
3. Build the code using `catkin_make`.
4. Open a terminal and run the following command to start the Gazebo simulator:
    roslaunch task3_env task3_env.launch
5. Open another terminal and run the following command to initiate the experiment:
    rosrun task3_env my_experiment.py > shell.txt 2>&1

## Additional Information

- **Experiment Execution:** Running `rosrun task3_env my_experiment.py` initiates an experiment consisting of 10 runs of picking balls.

- **Output Files:**
- `shell.txt`: Contains the shell commands and output of the experiment.
- `experiment_output.txt`: Stores data from all experiments, including the average rewards achieved by the robot.

- **Example Video:** A video demonstrating the experiment is available for reference.

---

_Guy Ginat, 206922544, and Ron Hadad, 209260645, completed this project._
