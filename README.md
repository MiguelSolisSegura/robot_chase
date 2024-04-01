# Robot Chase

The `robot_chase` repository contains a ROS2 package developed in C++ that enables interactive chase behavior between two robots, Rick and Morty, in a simulation environment. Utilizing TF (Transform Frames), the package calculates the relative positions of the robots to initiate the chase, dynamically adjusting Rick's movements to follow Morty.

A more comprehensive explanation of this project can be found [here](https://miguelsolissegura.com/project/barista-robots-p2).

## Getting Started

These instructions will guide you through setting up the `robot_chase` package on your local machine for development and execution.

### Prerequisites

- ROS2 Foxy Fitzroy or newer
- Gazebo simulation environment for testing the chase behavior
- The `barista_robot_description` package for robot models and simulation environment

### Installation

1. Clone the `barista_robot_description` repository (if not already done) and this `robot_chase` repository into your ROS2 workspace's `src` directory:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/MiguelSolisSegura/barista_robot_description.git
   git clone https://github.com/MiguelSolisSegura/robot_chase.git
   ```

2. Install any missing dependencies using `rosdep`:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the workspace to compile the `robot_chase` package:
   ```bash
   colcon build --packages-select robot_chase
   source install/setup.bash
   ```

## Running the Chase Simulation

Ensure you have launched the simulation environment with both robots present. If not, refer to the `barista_robot_description` repository's README for instructions on launching the robots in Gazebo.

1. With the simulation running, execute the `robot_chase` node in a new terminal to start the chase behavior:
   ```bash
   ros2 run robot_chase robot_chase
   ```

2. Control Morty's movements using the `teleop_twist_keyboard` package to test the chase dynamics. Ensure to remap the `cmd_vel` topic to Morty's namespace:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/morty/cmd_vel
   ```

Rick should now dynamically follow Morty based on the real-time transformations calculated by the `robot_chase` node.
