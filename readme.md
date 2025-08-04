# DYNAMIXEL ROS 2 Teleoperation with Joystick
This project enables teleoperation of DYNAMIXEL motors using a joystick controller via ROS 2


## Setup
**Source ROS2**
```bash
# If you haven't already, source your ROS2 installation
# example:
source /opt/ros/humble/setup.bash 
```
Add the [Humble dynamixel_sdk](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/humble/ros/dynamixel_sdk) to your ROS2 workspace

**Build the Dynamixel SDK**
```bash
colcon build --packages-select dynamixel_sdk
```

**Make sure rosdeps are up-to-date**
```bash
rosdep update
rosdep install --from-paths . # . to install all rosdeps for workspace if you are in workspace root
```

**Build dynajoy package**
```bash
# Since new dynamixel_sdk package has been built, source the updated local setup
source install/local_setup.bash 

# Build the package
colcon build --packages-select dynajoy
```

## Usage
```bash
ros2 launch dynajoy robot_launch.py
```

## TODO
The following items are listed in priority they should be done, with #1 being *required* for CIRC.
### 1. Proper Movement Based on Joystick (REQUIRED FOR CIRC)
Currently, moving the left joystick up/down moves both wrist motor positions to their corresponding values. For actual control, it is not this simple. For example, the wrist uses differential movement, so the two wrist motors must be coordinated to move as expected based on joystick input. The other motors must also be controlled.

**Tasks:**
 - Set reasonable default motor positions
 - Add control for all arm motors based on XBox controller inputs
 - Coordinate motors to move as expected based on controller input

### 2. Allow Override of Parameters
Currently, constants such as ``DEVICE_PORT`` are hardcoded even though their values are not necessarily static, requiring a rebuild whenever these values need changing.

These values should be overrideable as parameters for easier configuration.

For example:
```python
# dynajoy/command_publisher.py
def __init__(self):
    self.declare_parameter('device_port', "dev/ttyUSB0")
    ...
```

### 3. Split joy_callback
Right now, commands are sent every time joy_callback is run, which is very frequently and very inefficient. The ``command_publisher.py`` file should have functions for repeated actions, such as sending a position command or for coordinated movements based on controller input. 

These functions should only be called when there is an actual update. So, ``joy_callback`` will check if the controller values have actually updated and then either:
 - Call appropriate functions based on changed values
 - Update state variables with new values

If updating state variables instead of calling functions directly, then a timer callback can be used to then call the appropriate functions at a lower frequency than the joystick updates.

**Tasks**
 - Create functions for repeated or complex actions
 - Only call functions when values change or using a timer callback

