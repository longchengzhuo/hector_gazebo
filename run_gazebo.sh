#!/bin/bash

# Get the absolute path to the hector_gazebo workspace
HECTOR_GAZEBO_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Starting Hector Gazebo simulation from: $HECTOR_GAZEBO_WS"

# Step 0: Source Workspace Setup (in each terminal)
# Step 1: Launch Gazebo Simulation
gnome-terminal --tab --title="Gazebo Simulation" -- bash -c "
  cd "$HECTOR_GAZEBO_WS";
  source install/setup.bash;
  ign gazebo -r "$HECTOR_GAZEBO_WS/src/hector_gazebo/assets/models/worlds/world.sdf";
  exec bash" &

# Step 2: Run Hector Control Node
gnome-terminal --tab --title="Hector Control Node" -- bash -c "
  cd "$HECTOR_GAZEBO_WS";
  source install/setup.bash;
  ros2 launch hector_control control_launch.py;
  exec bash" &

# Step 3: Launch Keyboard Teleoperation
gnome-terminal --tab --title="Keyboard Teleop" -- bash -c "
  cd "$HECTOR_GAZEBO_WS";
  source install/setup.bash;
  ros2 launch keyboard_joy keyboard_joy.launch.py;
  exec bash" &

echo "Gazebo simulation launched in separate tabs. Check your terminal window for new tabs."
echo "Press Enter to close this initial script terminal."
read
