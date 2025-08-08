import rclpy

from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from enum import Enum
from numpy import interp

from dynamixel_sdk import COMM_SUCCESS
from dynamixel_sdk import PacketHandler
from dynamixel_sdk import PortHandler

DEVICE_PORT = "/dev/ttyUSB0"
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0

SHOULDER1_ID = 1
SHOULDER2_ID = 2
ELBOW_ID = 3
WRIST1_ID = 5
WRIST2_ID = 4
ADDR_GOAL_VELOCITY = 104
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132
ADDR_GOAL_POSITION = 116

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

MAX_VELOCITY = 30  # Adjust to your motor limits
MIN_VELOCITY = 0
MIN_POSITION=0
MAX_POSITION = 4095

class DynamixelTeleop(Node):
	def __init__(self):
		super().__init__('dynamixel_teleop')

		# Setup Joystick
		self.joy_sub = self.create_subscription(
			Joy,
			'/joy',
			self.joy_callback,
			10
		)

		# Setup Dynamixel
		self.port_handler = PortHandler(DEVICE_PORT)
		self.packet_handler = PacketHandler(PROTOCOL_VERSION)

		if not self.port_handler.openPort():
			self.get_logger().error("Failed to open port")
			return
		if not self.port_handler.setBaudRate(BAUDRATE):
			self.get_logger().error("Failed to set baudrate")
			return

		dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
			self.port_handler, WRIST1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
		)
		dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
			self.port_handler, WRIST2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
		)
		dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
			self.port_handler, ELBOW_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
		)
		dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
			self.port_handler, SHOULDER1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
		)
		dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
			self.port_handler, SHOULDER2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
		)

		if dxl_comm_result != COMM_SUCCESS:
			self.get_logger().error(f"Enable torque failed: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
		elif dxl_error != 0:
			self.get_logger().error(f"Dynamixel error: {self.packet_handler.getRxPacketError(dxl_error)}")

		self.get_logger().info("Dynamixel ready. Use joystick to control.")

	def joy_callback(self, joy_msg):
		x_axis = joy_msg.axes[0]  # Left/right
		y_axis = -joy_msg.axes[1]  # Forward/backward
		# Right joystick
		right_x = joy_msg.axes[3]      # Right stick left/right
		right_y = joy_msg.axes[4]      # Right stick up/down

		# D-pad
		dpad_x = joy_msg.axes[6]       # Left (-1), Right (+1)
		dpad_y = joy_msg.axes[7]       # Down (-1), Up (+1)
		l_bumper = joy_msg.buttons[4]


		# TODO: Gotta do math stuff so wrist motors move properly
		# issue of creating individual dynamixel movement, rather than one input moving both motors at the same time,
		# which does not allow rotation, only up and down movement 
		velocity_elbow = int(interp(dpad_x, [-1, 1], [MIN_VELOCITY, MAX_VELOCITY])) # this works as there is only one motor on this particular joint
		if bool(l_bumper):
			position_wrist1 = int(interp(x_axis, [-1, 1], [MIN_POSITION, MAX_POSITION]))
			position_wrist2 = 0 # or position_wrist2 = int(interp(left_y, [-1, 1], [MIN_POSITION, MAX_POSITION]))
			velocity_shoulder1 = int(interp(right_x, [-1, 1], [MIN_VELOCITY, MAX_VELOCITY]))
			position_shoulder2 = 0

		else:
			position_wrist1 = int(interp(y_axis, [-1, 1], [MIN_POSITION, MAX_POSITION]))
			position_wrist2 = int(interp(y_axis, [-1, 1], [MIN_POSITION, MAX_POSITION]))
			velocity_shoulder1 = int(interp(right_y, [-1, 1], [MIN_VELOCITY, MAX_VELOCITY]))
			velocity_shoulder2 = int(interp(right_y, [-1, 1], [MIN_VELOCITY, MAX_VELOCITY]))

		# Write velocity to motor
		dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
			self.port_handler, WRIST1_ID, ADDR_GOAL_POSITION, self._to_twos_complement(position_wrist1, 4)
		)
		dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
			self.port_handler, WRIST2_ID, ADDR_GOAL_POSITION, self._to_twos_complement(position_wrist2, 4)
		)
		dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
			self.port_handler, ELBOW_ID, ADDR_GOAL_VELOCITY, self._to_twos_complement(velocity_elbow, 4)
		)
		dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
			self.port_handler, SHOULDER1_ID, ADDR_GOAL_VELOCITY, self._to_twos_complement(velocity_shoulder1, 4)
		)
		dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
			self.port_handler, SHOULDER2_ID, ADDR_GOAL_VELOCITY, self._to_twos_complement(velocity_shoulder2, 4)
		)


		if dxl_comm_result != COMM_SUCCESS:
			self.get_logger().error(f"Write Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
		elif dxl_error != 0:
			self.get_logger().error(f"Dynamixel error: {self.packet_handler.getRxPacketError(dxl_error)}")
		else:
			self.get_logger().info(f"Position Command Sent for Wrist 1: {position_wrist1}")
			self.get_logger().info(f"Position Command Sent for Wrist 2: {position_wrist2}")
			self.get_logger().info(f"Velocity Command Sent for Elbow : {velocity_elbow}")
			self.get_logger().info(f"Velocity Command Sent for Shoulder 1 : {velocity_shoulder1}")
			self.get_logger().info(f"Velocity Command Sent for Shoulder 2 : {velocity_shoulder2}")


	def _to_twos_complement(self, value, byte_size):
		max_val = 2 ** (byte_size * 8)
		return value if value >= 0 else max_val + value

def main(args=None):
	rclpy.init(args=args)
	node = DynamixelTeleop()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()