#!/usr/bin/env bash

# Move to workspace directory
WORKSPACE_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$WORKSPACE_DIR"

# 1. Clean up any stale simulator/ROS processes from previous runs
echo "==> Cleaning up background processes..."
pkill -9 -f gz 2>/dev/null
pkill -9 -f ros2 2>/dev/null
pkill -9 -f rviz2 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
pkill -9 -f ros_gz_bridge 2>/dev/null

# 2. Source ROS 2 Jazzy underlay
echo "==> Sourcing ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

# 3. Build workspace
echo "==> Building Workspace..."
colcon build --symlink-install

# 4. Source workspace overlay
echo "==> Sourcing Workspace Overlay..."
source install/setup.bash

# 5. Open a new terminal window for Multi-Key Teleop
echo "==> Launching Multi-Key Teleop Window..."
gnome-terminal --title="Flipper Car TC1508 Teleop" -- bash -c "
  source /opt/ros/jazzy/setup.bash
  source '$WORKSPACE_DIR/install/setup.bash'
  echo 'Waiting for simulation & bridge to initialize...'
  sleep 4
  ros2 run flipper_car teleop_keyboard
  exec bash
"

# 6. Launch Gazebo, Bridges, Controller, Tracker, and RViz
echo "==> Starting Simulation, Bridge, Controller, Tracker & RViz..."
ros2 launch flipper_car flipper_car.launch.py