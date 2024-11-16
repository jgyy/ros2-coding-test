# ros2_interview_ws

ros2_interview_ws

```sh
# Create workspace
mkdir -p ~/ros2_interview_ws/src
cd ~/ros2_interview_ws/src

# Create packages
ros2 pkg create --build-type ament_python basic_pubsub
ros2 pkg create --build-type ament_python robot_monitor

# Copy the code files to their respective locations
# Build the workspace
cd ~/ros2_interview_ws
colcon build

# Source the workspace
source install/setup.bash

# Launch the system
ros2 launch system.launch.py
```
