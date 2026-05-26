import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

deadzone    = 0.1
sensitivity = 1.0

# robot_params.yaml is installed alongside the package (see setup.py)
_params_file = os.path.join(
    get_package_share_directory('dynajoy'),
    'config',
    'robot_params.yaml',
)


def generate_launch_description():
    return LaunchDescription([

        # ── Joystick driver ───────────────────────────────────────────────────
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node',
            parameters=[
                {'controller_id': 0},
                {'deadzone': deadzone},
                {'sensitivity': sensitivity},
            ],
            remappings=[('/joy', '/controller1/joy')],
        ),

        # ── Arm controller ────────────────────────────────────────────────────
        # robot_params.yaml is loaded via the ROS 2 parameter server.
        # All values are visible and changeable at runtime:
        #   ros2 param list /command_publisher
        #   ros2 param set  /command_publisher ik.step_mm 8.0
        Node(
            package='dynajoy',
            executable='command_publisher',
            name='command_publisher',
            parameters=[
                _params_file,                       # ← robot_params.yaml
                {'controller_id': 1},
                {'deadzone': deadzone},
                {'sensitivity': sensitivity},
            ],
            remappings=[('/joy', '/controller1/joy')],
        ),
    ])
