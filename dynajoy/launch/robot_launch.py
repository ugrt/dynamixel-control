import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

deadzone = 0.1
sensitivity = 1.0

def generate_launch_description():
	return LaunchDescription([
		# Rover Controller
		Node(
			package='joy_linux',
			executable='joy_linux_node',
			name='joy_linux_node',
			parameters=[
				{'controller_id': 0},  # e.g., /dev/input/js0
				{'deadzone': deadzone},
				{'sensitivity': sensitivity}
			],
			remappings=[
				('/joy', '/controller1/joy'),
			]
		),

		# Arm Controller
		Node(
			package='dynajoy',
			executable='command_publisher',
			name='command_publisher',
			parameters=[
				{'controller_id': 1},  # e.g., /dev/input/js1
				{'deadzone': deadzone},
				{'sensitivity': sensitivity}
			],
			remappings=[
				('/joy', '/controller1/joy'),
			]
		),
		Node(
			package='dynajoy',
			executable='grip_control',
			name='grip_control',
			parameters=[
				{'controller_id': 1},  # e.g., /dev/input/js1
				{'deadzone': deadzone},
				{'sensitivity': sensitivity}
			],
			remappings=[
				('/joy', '/controller1/joy'),
			]
		),
	])

#def generate_launch_description():
 #   package_dir = get_package_share_directory('dynajoy')
  #  command_publisher = Node(
   #     package='dynajoy',
	#    executable='command_publisher',
	#)
#
 #   return LaunchDescription([
  #      command_publisher,
   #     launch.actions.RegisterEventHandler(
	#        event_handler=launch.event_handlers.OnProcessExit(
	 #           on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
	  #      )
	   # )
	#])




