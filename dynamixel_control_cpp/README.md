# Dynamixel Control C++ Project

## Overview
The Dynamixel Control C++ project is designed to control Dynamixel motors using joystick input. It is built on ROS 2 and provides a C++ implementation of the functionality originally developed in Python. The project includes a launch file to set up the necessary nodes and a command publisher to handle motor commands based on joystick movements.

## Project Structure
```
dynamixel_control_cpp
├── CMakeLists.txt          # CMake configuration file
├── package.xml             # ROS 2 package metadata
├── launch
│   └── robot_launch.cpp    # Launch configuration for ROS 2 nodes
├── src
│   └── command_publisher.cpp # Implementation of the DynamixelTeleop class
├── include
│   └── dynamixel_control_cpp
│       └── command_publisher.hpp # Header file for the DynamixelTeleop class
└── README.md               # Project documentation
```

## Setup Instructions
1. **Install ROS 2**: Ensure that you have ROS 2 installed on your system. Follow the official installation guide for your platform.

2. **Clone the Repository**: Clone this repository to your ROS 2 workspace.
   ```bash
   git clone <repository-url>
   ```

3. **Build the Project**: Navigate to your ROS 2 workspace and build the project using colcon.
   ```bash
   cd <your_ros2_workspace>
   colcon build --packages-select dynamixel_control_cpp
   ```

4. **Source the Workspace**: After building, source the workspace to make the package available.
   ```bash
   source install/setup.bash
   ```

## Usage
To launch the nodes, use the following command:
```bash
ros2 launch dynamixel_control_cpp robot_launch.cpp
```

## Dependencies
- `joy_linux`: For joystick input handling.
- `dynamixel_sdk`: For communication with Dynamixel motors.

## Author
- [Your Name] - [Your Email]

## License
This project is licensed under the MIT License. See the LICENSE file for more details.