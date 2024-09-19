
# Pick and Place Spring 2022 Setup

This repository sets up the simulation environment for the Pick and Place Spring 2022 project.

## Prerequisites

Ensure that you have the following installed:
- ROS Melodic
- Gazebo
- catkin tools

## Installation

1. Navigate to the `src` directory of your catkin workspace:
    ```bash
    cd ~/catkin_ws/src
    ```

2. Clone the repository:
    ```bash
    git clone <repository-url>
    ```

3. Edit your `.bashrc` file to set the Gazebo model path and add an alias for easy launching:
    ```bash
    nano ~/.bashrc
    ```

4. Add the following lines to the end of the file:
    ```bash
    export GAZEBO_MODEL_PATH=/absolute/path/to/pickandplace_spring2022/workcell_809e/models:$GAZEBO_MODEL_PATH
    alias kittingrqt='rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller robot_description:=/ariac/kitting/robot_description'
    ```

5. Source your updated `.bashrc` file:
    ```bash
    source ~/.bashrc
    ```

## Build the Workspace

1. Navigate to the repository folder:
    ```bash
    cd ~/catkin_ws/src/pickandplace_spring2022
    ```

2. Install dependencies:
    ```bash
    rosdep install --from-paths . --ignore-src --rosdistro melodic -y
    ```

3. Build the workspace:
    ```bash
    catkin build
    ```

4. Source your `.bashrc` file again:
    ```bash
    source ~/.bashrc
    ```

## Running the Simulation

Check that the simulation environment loads correctly with:
```bash
roslaunch workcell_809e workcell.launch
```

## License

[Include your license information here, if applicable.]

---

This project is developed as part of the Pick and Place Spring 2022 course.
