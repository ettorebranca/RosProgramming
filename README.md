# Simple Planner RP

## Overview

This project implements a simple path planning system using ROS Noetic, designed to work with RViz for visualization and map_server for handling occupancy grids.

## Features

- **Occupancy Grid Handling**: Subscribes to the `/map` topic to receive the occupancy grid from the map server.
- **Initial Position**: Uses RViz's “2D Pose Estimate” tool to set the robot's initial position.
- **Path Planning**: Implements a basic A* algorithm to plan a path from the start position to the goal.
- **RViz Integration**: Publishes the planned path and robot's position to RViz for visualization.

### Prerequisites

- **ROS Noetic**: Ensure ROS Noetic is installed and properly set up
  ```bash
  sudo apt install ros-noetic
- **RViz**: For visualization.
  ```bash
  sudo apt install rviz
- **map_server**: To handle the occupancy grid.
  ```bash
  sudo apt install map-server

### Installation

1. Clone the repository to your catkin workspace.
   ```bash
   git clone https://github.com/ettorebranca/RosProgramming
2. Build the workspace:
   ```bash
   cd ~/catkin_ws
   catkin_make
3. Source the setup file:
   ```bash
   source devel/setup.bash
4. Start the ROS master:
   ```bash
   roscore
5. Run the launch file:
   ```bash
   bash run.sh
6. Open RViz and use the "2D Pose Estimate" tool to set the robot's initial position.
7. Monitor the robot's path and position in RViz.


### Project Structure

- **src/**: Contains source files for the (main) node, RViz integration, path finding, and map management.
- **launch/**: Contains the sp.launch file to start all required nodes.
- **maps/**: Contains the map files (map.pgm, map.yaml).

   
  


