# ROS2 Gazebo Line Follower Robot

## Overview
This project is a simple ROS2 implementation of a Gazebo-simulated robot that follows a line using computer vision techniques in Python. It's designed for those interested in robotics, simulation, and autonomous navigation systems.

## Features
- **ROS2 Integration:** Leverages the power of ROS2 for messaging and system management.
- **Gazebo Simulation:** Utilizes the Gazebo simulator for a realistic robotics simulation environment.
- **Computer Vision:** Employs Python-based computer vision algorithms to detect and follow lines.

## Prerequisites
- ROS2 Foxy Fitzroy or newer
- Gazebo simulator
- Python 3.6 or newer
- OpenCV for Python

## Installation
1. **Install ROS2:** Follow the official ROS2 documentation to install ROS2 on your system.
2. **Install Gazebo:** Ensure Gazebo is installed and properly configured with ROS2.
3. **Clone the Repository:** Clone this repository into your ROS2 workspace's `src` directory.
4. **Install Dependencies:** Run `rosdep install --from-paths src --ignore-src -r -y` in your workspace directory to install required dependencies.
5. **Build the Workspace:** Use `colcon build` to build your ROS2 workspace.

## Running the Simulation
1. **Launch Gazebo:** Start the Gazebo simulator with the provided world file.
2. **Run the Line Follower Node:** Execute the line follower node to start the robot.
3. **Visualize in RViz:** Optionally, use RViz to visualize the robot's sensors and path.

## Node Details
- **`line_follower_node`**: The main node that implements the line following logic using computer vision.

## License
This project is licensed under the MIT License - see the `LICENSE.md` file for details.

## Acknowledgments
- The ROS2 community for providing an excellent framework for robotics development.
- The Gazebo team for creating a powerful simulation tool.
- Contributors to the OpenCV project for their work on computer vision.