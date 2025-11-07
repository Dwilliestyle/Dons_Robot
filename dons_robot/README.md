# RaspRover ROS2 Package

ROS2 control package for Waveshare RaspRover with 4-wheel, 4WD system.

## Features
- ESP32 serial bridge for motor control
- Joystick teleop control with dual speed modes
- URDF model with STL meshes
- Camera integration
- D500 LIDAR support
- SLAM mapping capability

## Hardware
- Waveshare RaspRover (4-wheel, 4WD)
- Raspberry Pi 5
- ESP32 sub-controller (serial communication)
- D500 LiDAR Kit
- Camera without pan-tilt mount

## Package Contents
- **esp32_bridge.py**: Serial communication node for motor control
- **launch/**: Various launch files for different robot modes
- **urdf/**: Robot description files
- **meshes/**: STL files for visualization
- **config/**: Configuration files for various components

## Installation
```bash
# Clone repository
cd ~/dons_robot_ws/src
git clone https://github.com/Dwilliestyle/dons_robot.git

# Build
cd ~/dons_robot_ws
colcon build --packages-select dons_robot
source install/setup.bash
```

## Usage
```bash
# Launch robot with teleop
ros2 launch dons_robot robot_teleop.launch.py

# View robot in RViz
ros2 launch dons_robot view_robot.launch.py

# Start SLAM mapping
ros2 launch dons_robot slam_mapping.launch.py
```

## Serial Communication
- Port: /dev/ttyAMA0
- Baud: 115200
- Protocol: JSON commands to ESP32

## Author
Don
