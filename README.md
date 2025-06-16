# Autonomous-Mapping-Navigation-and-Search-with-TurtleBot3

## Overview

This individual project showcases a complete autonomous robotic pipeline built using **TurtleBot3**, **ROS2**, and **Gazebo**, integrating concepts learned throughout ME 597. The system was developed in a simulated home-like environment with a focus on:

- Autonomous mapping through frontier exploration
- Path planning and navigation with obstacle avoidance
- Visual search and localization of dynamic objects

The project was executed in a **modified TurtleBot3 house world**, providing diverse spatial features and navigation challenges to test robotic behavior.

---

## Project Components

### 🔹 Task 1 – Autonomous Mapping

Designed a frontier-based mapping system using:

- **Occupancy grid + WSD** algorithm for frontier detection
- **A\*** path planning for efficient exploration
- **Odometry** for localization

The TurtleBot autonomously explored the entire environment within a 10-minute time constraint, achieving **full coverage**.

### 🔹 Task 2 – Navigation with Static Obstacles

Implemented a global path navigation system with:

- **AMCL** for robot localization  
- **A\*** planner using the map generated in Task 1  
- Dynamic obstacle detection by comparing **real-time LiDAR data** with the static loaded map

The robot avoided unmapped, randomly placed trash cans without collisions while reaching sequential goals.  


### 🔹 Task 3 – Search and Localize

Developed a computer vision and search pipeline to:

- Detect **randomly spawned colored balls** using **OpenCV**
- Classify color and approximate **(x, y)** coordinates
- Maintain **collision-free exploration** using Task 2’s obstacle avoidance

Achieved:
- 100% color classification accuracy  
- 99.8% accuracy on x-coordinate  
- 87.8% accuracy on y-coordinate  

---
## Prerequisites

- **ROS2 Humble** – middleware framework for robot control and communication
- **Gazebo** – 3D physics simulation environment
- **RViz** – visualization tool for robot state and environment
- **Python** – core programming language for nodes and algorithms
- **OpenCV** – computer vision for object detection and color classification

---
## Setup Instructions

### 1. Clone the Simulation Workspace

```bash
cd ~/ros2  # or any directory you prefer
git clone https://github.com/AlbertWeirdo/Autonomous-Mapping-Navigation-and-Search-with-TurtleBot3-.git
cd Autonomous-Mapping-Navigation-and-Search-with-TurtleBot3
```

### 2. Build the Workspace

```bash
colcon build --symlink-install
```

### 3. Set TurtleBot3 Model

Add this to your `.bashrc` to avoid re-exporting:

```bash
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

### 4. Install Required Packages

Install simulation and navigation dependencies:

```bash
sudo apt install ros-humble-turtlebot3-teleop
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-navigation2
```

Install Python library:

```bash
pip install pynput
```

### 5. Source the Workspace (Every Session)

```bash
cd ~/ros2/Autonomous-Mapping-Navigation-and-Search-with-TurtleBot3
source install/local_setup.bash
```

---

### How to Run the Simulation & Tasks
Before launching any task, always make sure to:

```bash
cd ~/ros2/Autonomous-Mapping-Navigation-and-Search-with-TurtleBot3         # or your workspace path
colcon build --packages-select turtlebot3_gazebo            # build all updated packages
source install/setup.bash
```


## Running Each Task

### Task 1 – Autonomous Mapping

```bash
ros2 launch turtlebot3_gazebo mapper.launch.py
```

Runs SLAM and `task1.py` to autonomously explore the environment.

Once exploration is complete, **manually save the map** with:

```bash
ros2 run nav2_map_server map_saver_cli -f "map_name"
```

This will generate `map_name.pgm` and `map_name.yaml`.
[Official TurtleBot Map Saving Guide](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html#save-the-map)

### Task 2 – Navigation with Static Obstacles

```bash
ros2 launch turtlebot3_gazebo navigator.launch.py static_obstacles:=true
```

Loads the map previously saved in Task 1 and runs `task2.py` using AMCL localization and A* global path planning.  
Random trash cans are spawned as unmapped static obstacles for obstacle avoidance testing.

---

### Task 3 – Object Search & Localization

```bash
ros2 launch turtlebot3_gazebo navigator.launch.py spawn_objects:=true
```

Reuses the map generated in Task 1 and runs `task3.py` to detect, classify, and localize randomly spawned colored balls using OpenCV.

---

### Reference Repository & Base Code

- **Official Project Description**  
  (https://github.com/Purdue-ME597/ME597-Spring2025/blob/main/5-Final_Project/1-Turtlebot_Instructions.md)  
  

- **Simulation Workspace Base Code**  
  (https://github.com/naslab-projects/sim_ws/tree/main)  

---