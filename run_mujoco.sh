#!/bin/bash

# Get the absolute path to the hector_gazebo workspace
HECTOR_GAZEBO_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Starting Hector MuJoCo simulation from: $HECTOR_GAZEBO_WS"

# Step 0 & MuJoCo specific command
gnome-terminal --tab --title="MuJoCo Node" -- bash -c "
  cd "$HECTOR_GAZEBO_WS";
  source install/setup.bash;
  ros2 run hector_mujoco hector_mujoco_node;
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

echo "MuJoCo simulation launched in separate tabs. Check your terminal window for new tabs."
echo "Press Enter to close this initial script terminal."
read
