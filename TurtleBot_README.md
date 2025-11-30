# TurtleBot3 Obstacle Detection and Avoidance

A ROS Noetic implementation of autonomous obstacle avoidance for TurtleBot3 using LiDAR sensors in Gazebo simulation.

## Overview

This project implements a behavior where the TurtleBot3 can be controlled via keyboard teleop, but automatically overrides user commands to avoid obstacles when detected within a 30-degree cone in front of the robot.

### Key Features

- **LiDAR-based obstacle detection** using `/scan` topic
- **Automatic collision avoidance** with right-turn maneuver
- **Teleop override mechanism** that returns control to user when path is clear
- **Real-time range data publishing** for monitoring and debugging

## System Requirements

- **ROS Version:** ROS1 Noetic
- **Operating System:** Ubuntu 20.04 (or Docker with `osrf/ros:noetic-desktop-full`)
- **Simulation:** Gazebo
- **Robot Model:** TurtleBot3 Burger

## Dependencies

Install required ROS packages:

```bash
sudo apt-get install ros-noetic-turtlebot3 \
                     ros-noetic-turtlebot3-simulations \
                     ros-noetic-turtlebot3-msgs
```

## Installation

1. Clone this repository into your catkin workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/ApurvK032/TurtleBot3-ROS-Obstacle-Detection-Avoidance.git hw_pkg
```

2. Build the workspace:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

3. Set the TurtleBot3 model:
```bash
export TURTLEBOT3_MODEL=burger
```

## Usage

### Running the Simulation

Launch everything with a single command:
```bash
roslaunch hw_pkg hw2.launch
```

This starts:
- Gazebo simulation with TurtleBot3
- Keyboard teleop control
- Obstacle detection node
- Obstacle avoidance node

### Keyboard Controls

- `w` - Move forward
- `x` - Move backward
- `a` - Turn left
- `d` - Turn right
- `s` or `spacebar` - Emergency stop

### Behavior

The robot follows keyboard commands normally. When an obstacle is detected within **0.5 meters** in a **±15° cone** in front, the robot:
1. Stops forward motion
2. Rotates right at 0.5 rad/s
3. Continues until obstacle is cleared
4. Returns control to teleop

## Node Architecture

### obstacle_detection.py

- **Subscribes to:** `/scan` (sensor_msgs/LaserScan)
- **Publishes to:** `/ranges` (std_msgs/Float32MultiArray)
- **Purpose:** Extracts and republishes LiDAR range data for monitoring

### obstacle_avoidance.py

- **Subscribes to:** 
  - `/scan` (sensor_msgs/LaserScan)
  - `/teleop_cmd_vel` (geometry_msgs/Twist)
- **Publishes to:** `/cmd_vel` (geometry_msgs/Twist)
- **Purpose:** Implements obstacle detection logic and teleop override

**Detection Algorithm:**
1. Reads 360° LiDAR scan
2. Filters front cone (±15° from center)
3. Checks minimum valid distance
4. Triggers avoidance if distance < 0.5m

## Monitoring

Check published range data:
```bash
rostopic echo /ranges
```

View active nodes:
```bash
rosnode list
```

## Docker Setup (Optional)

If running on Ubuntu 24.04 or other non-compatible systems:

```bash
xhost +local:docker
docker run -it \
  --name ros_turtlebot \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/catkin_ws:/home/catkin_ws \
  osrf/ros:noetic-desktop-full
```

Inside container:
```bash
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
export TURTLEBOT3_MODEL=burger
source /home/catkin_ws/devel/setup.bash
```

## Project Structure

```
hw_pkg/
├── launch/
│   └── hw2.launch          # Master launch file
├── scripts/
│   ├── obstacle_detection.py   # Range publisher
│   └── obstacle_avoidance.py   # Avoidance controller
├── CMakeLists.txt
└── package.xml
```

## Technical Details

**Obstacle Detection Cone:** 30° total (±15° from robot heading)  
**Trigger Distance:** 0.5 meters  
**Avoidance Behavior:** Stop + rotate right at -0.5 rad/s  
**LiDAR Resolution:** 360 readings per scan  

## License

MIT License

## Author

Apurv Kushwaha
