#!/usr/bin/env bash

# Move to workspace directory
cd "$(dirname "$0")"

# 1. Clean up any stale simulator/ROS processes from previous runs
echo "==> Cleaning up background processes..."
pkill -9 -f gz 2>/dev/null
pkill -9 -f ros2 2>/dev/null
pkill -9 -f rviz2 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null

# 2. Source ROS 2 Jazzy underlay
echo "==> Sourcing ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

# 3. Build workspace
echo "==> Building Workspace..."
colcon build --symlink-install

# 4. Source workspace overlay
echo "==> Sourcing Workspace Overlay..."
source install/setup.bash

# 5. Launch Gazebo, Bridges, Robot State, Controller, and RViz 2
echo "==> Starting Simulation & RViz..."
ros2 launch flipper_car flipper_car.launch.py