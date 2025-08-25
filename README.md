# RRT Path Planning in ROS

This project implements Rapidly-exploring Random Tree (RRT) path planning for autonomous navigation in ROS. The implementation includes a complete ROS package with map handling, trajectory generation, and visualization capabilities.

## Project Overview

The system consists of three main Python modules:
- **RRT_node.py**: Main ROS node that handles map data and performs path planning
- **motion_planner.py**: User interface node for inputting start/goal coordinates
- **rrt.py**: Core RRT algorithm implementation

## Prerequisites

### System Requirements
- **VMware Workstation** (as used in original development)
- **Ubuntu** (recommended: 18.04 or 20.04)
- **ROS Melodic** or **ROS Noetic**
- **Python 3**

## Installation and Setup

### 1. Setup ROS Workspace
```bash
# Build the workspace
catkin_make

# Source the workspace
source devel/setup.bash
```

### 2. Make Python Scripts Executable
```bash
chmod +x src/rrt_pathfinding/src/*.py
```

## Usage

### Step 1: Start ROS Core
```bash
roscore
```

### Step 2: Load Map Server
```bash
rosrun map_server map_server my_map.yaml
```

### Step 3: Start RRT Planning Node
```bash
rosrun rrt_pathfinding RRT_node.py
```

### Step 4: Start Motion Planner Interface
```bash
rosrun rrt_pathfinding motion_planner.py
```

### Step 5: Input Coordinates
The motion planner will prompt you to enter:
- Start X coordinate (meters)
- Start Y coordinate (meters)  
- Goal X coordinate (meters)
- Goal Y coordinate (meters)

Example input:
```
Enter start X (m): -2.0
Enter start Y (m): -0.5
Enter goal X (m): 2.0
Enter goal Y (m): 1.0
```

## System Architecture

### ROS Topics
- `/map`: OccupancyGrid message containing the map data
- `/start_goal`: Float64MultiArray with [start_x, start_y, goal_x, goal_y]
- `/trajectory`: Float64MultiArray with flattened waypoint coordinates
- `/path_ready`: Bool indicating when path planning is complete

### Node Communication Flow
1. Map server publishes map on `/map` topic
2. User enters start/goal coordinates via motion_planner
3. Motion planner publishes coordinates on `/start_goal`
4. RRT_node receives coordinates and computes path
5. RRT_node publishes trajectory on `/trajectory`
6. RRT_node signals completion via `/path_ready`
7. Visualization image is saved automatically

## Output

### Trajectory Visualization
- Generated PNG images show:
  - **Green circle (S)**: Start position
  - **Blue circle (G)**: Goal position  
  - **Red line**: Computed RRT path
- Files named: `s(start_x_start_y)_g(goal_x_goal_y).png`

## Map Configuration

The included map (`my_map.yaml`) has the following properties:
- **Resolution**: 0.05 meters/pixel
- **Origin**: (-10.0, -10.0) meters
- **Size**: 400×400 pixels (20×20 meter world)