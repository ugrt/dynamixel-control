import rclpy

from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from dynamixel_sdk import *
from enum import Enum
from numpy import interp

DEVICENAME = "/dev/ttyUSB0"
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0

DXL_ID = 1
ADDR_GOAL_VELOCITY = 104
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132
ADDR_GOAL_POSITION = 116

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

MAX_VELOCITY = 300  # Adjust to your motor limits

class DynamixelTeleop(Node):
    def __init__(self):
        super().__init__('dynamixel_teleop')

        # Setup Joystick
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        # Setup Dynamixel
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error("Failed to open port")
            return
        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set baudrate")
            return

        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Enable torque failed: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            self.get_logger().error(f"Dynamixel error: {self.packet_handler.getRxPacketError(dxl_error)}")

        self.get_logger().info("Dynamixel ready. Use joystick to control.")

    def joy_callback(self, joy_msg):
        x_axis = joy_msg.axes[0]  # Left/right
        y_axis = -joy_msg.axes[1]  # Forward/backward

        velocity = int(y_axis * MAX_VELOCITY)

        # Write velocity to motor
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, DXL_ID, ADDR_GOAL_VELOCITY, self._to_twos_complement(velocity, 4)
        )

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Write Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            self.get_logger().error(f"Dynamixel error: {self.packet_handler.getRxPacketError(dxl_error)}")
        else:
            self.get_logger().info(f"Velocity Command Sent: {velocity}")

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