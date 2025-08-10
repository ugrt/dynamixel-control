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

import RPi.GPIO as GPIO


DEVICE_PORT = "/dev/ttyUSB0"
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0

SHOULDER1_ID = 1
SHOULDER2_ID = 2
ELBOW_ID = 3
WRIST1_ID = 5
WRIST2_ID = 4
GRIPPER_ID = 6
ADDR_GOAL_VELOCITY = 104
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132
ADDR_GOAL_POSITION = 116
ADDR_GOAL_PWM = 100 # Limit the speed

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

MAX_VELOCITY = 100  # Adjust to your motor limits
MIN_VELOCITY = -100
MIN_POSITION=0
MAX_POSITION = 3500
MAX_PWM = 250

# Default motor start positions
RESTING_POINT = 1750
# position_wrist1 = RESTING_POINT
# position_wrist2 = RESTING_POINT
velocity_wrist1 = 0
velocity_wrist2 = 0
velocity_shoulder1 = 0
velocity_shoulder2 = 0
velocity_elbow = 0

deadzone = 50


class DRV8831PWMController(Node):
    def __init__(self):
        super().__init__('drv8831_pwm_controller')

        # Motor control pins (BCM numbering)
        self.IN1_PIN = 12  # Physical pin 32
        self.IN2_PIN = 13  # Physical pin 33

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.IN1_PIN, GPIO.OUT)
        GPIO.setup(self.IN2_PIN, GPIO.OUT)

        # PWM setup
        self.freq_hz = 1000  # Adjust for your motor
        self.pwm_in1 = GPIO.PWM(self.IN1_PIN, self.freq_hz)
        self.pwm_in2 = GPIO.PWM(self.IN2_PIN, self.freq_hz)
        self.pwm_in1.start(0)
        self.pwm_in2.start(0)

        # ROS 2 subscription for motor speed (-100 to 100)
        self.subscription = self.create_subscription(
            Float32,
            'drv8831_motor_speed',
            self.motor_callback,
            10
        )

        self.get_logger().info("DRV8831 PWM controller started on GPIO12 (IN1) and GPIO13 (IN2).")

    def motor_callback(self, msg: Float32):
        speed = max(-100.0, min(100.0, msg.data))  # Clamp speed to [-100, 100]

        if speed > 0:
            # Forward: PWM on IN1, IN2 LOW
            self.pwm_in1.ChangeDutyCycle(abs(speed))
            self.pwm_in2.ChangeDutyCycle(0)
        elif speed < 0:
            # Reverse: PWM on IN2, IN1 LOW
            self.pwm_in1.ChangeDutyCycle(0)
            self.pwm_in2.ChangeDutyCycle(abs(speed))
        else:
            # Stop motor
            self.pwm_in1.ChangeDutyCycle(0)
            self.pwm_in2.ChangeDutyCycle(0)

    def destroy_node(self):
        self.pwm_in1.stop()
        self.pwm_in2.stop()
        GPIO.cleanup()
        super().destroy_node()

class DynamixelTeleop(Node):
    def __init__(self):
        super().__init__('dynamixel_teleop')

        # Setup Joystick
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            1
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
        
        # TODO: Add disable Torque first for initialization reset

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

        # Set max movement speed
        # dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
        # 	self.port_handler, WRIST1_ID, ADDR_GOAL_PWM, MAX_PWM
        # )
        # dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
        # 	self.port_handler, WRIST2_ID, ADDR_GOAL_PWM, MAX_PWM
        # )

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Enable torque failed: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            self.get_logger().error(f"Dynamixel error: {self.packet_handler.getRxPacketError(dxl_error)}")

        self.get_logger().info("Dynamixel ready. Use joystick to control.")

    def joy_callback(self, joy_msg):
        global velocity_wrist1, velocity_wrist2, velocity_elbow, velocity_shoulder1, velocity_shoulder2, velocity_elbow
        x_axis = joy_msg.axes[0]  # Left/right
        y_axis = -joy_msg.axes[1]  # Forward/backward
        # Right joystick
        right_x = joy_msg.axes[3]      # Right stick left/right
        right_y = joy_msg.axes[4]      # Right stick up/down

        # D-pad
        dpad_x = joy_msg.axes[6]       # Left (-1), Right (+1)
        dpad_y = joy_msg.axes[7]       # Down (-1), Up (+1)
        l_bumper = joy_msg.buttons[4]

        # Buttons
        torque_reset = joy_msg.buttons[1]

        # New: Read right bumper and right trigger
        right_bumper = joy_msg.buttons[5]  # RB
        right_trigger = joy_msg.axes[5]    # RT analog value

        if right_bumper:
            # Close gripper
            self.packet_handler.write4ByteTxRx(
                self.port_handler, GRIPPER_ID, ADDR_GOAL_VELOCITY,
                self._to_twos_complement(MAX_VELOCITY, 4)
            )
        elif right_bumper:
            # Open gripper
            self.packet_handler.write4ByteTxRx(
                self.port_handler, GRIPPER_ID, ADDR_GOAL_VELOCITY,
                self._to_twos_complement(-MAX_VELOCITY, 4)
            )
        else:
            # Stop gripper
            self.packet_handler.write4ByteTxRx(
                self.port_handler, GRIPPER_ID, ADDR_GOAL_VELOCITY, 0
            )
        # Reset torque
        if torque_reset:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, WRIST1_ID, ADDR_TORQUE_ENABLE, not TORQUE_ENABLE
            )
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, WRIST2_ID, ADDR_TORQUE_ENABLE, not TORQUE_ENABLE
            )
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, ELBOW_ID, ADDR_TORQUE_ENABLE, not TORQUE_ENABLE
            )
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, SHOULDER1_ID, ADDR_TORQUE_ENABLE, not TORQUE_ENABLE
            )
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, SHOULDER2_ID, ADDR_TORQUE_ENABLE, not TORQUE_ENABLE
            )

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


        # TODO: Gotta do math stuff so wrist motors move properly
        # issue of creating individual dynamixel movement, rather than one input moving both motors at the same time,
        # which does not allow rotation, only up and down movement 
        velocity_elbow = int(interp(dpad_x, [-1, 1], [MIN_VELOCITY, MAX_VELOCITY])) # this works as there is only one motor on this particular joint
        if bool(l_bumper):
            velocity_wrist1 = int(interp(y_axis, [-1, 1], [MIN_VELOCITY, MAX_VELOCITY]))
            velocity_wrist2 = int(interp(-y_axis, [-1, 1], [MIN_VELOCITY, MAX_VELOCITY]))
            velocity_shoulder1 = int(interp(right_y, [-1, 1], [MIN_VELOCITY, MAX_VELOCITY]))
            velocity_shoulder2 = 0
        else:
            velocity_wrist1 = int(interp(y_axis, [-1, 1], [MIN_VELOCITY, MAX_VELOCITY]))
            velocity_wrist2 = int(interp(y_axis, [-1, 1], [MIN_VELOCITY, MAX_VELOCITY]))
            velocity_shoulder1 = int(interp(right_y, [-1, 1], [MIN_VELOCITY, MAX_VELOCITY]))
            velocity_shoulder2 = int(interp(right_y, [-1, 1], [MIN_VELOCITY, MAX_VELOCITY]))

        # Write velocity to motor
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
        	self.port_handler, WRIST1_ID, ADDR_GOAL_VELOCITY, self._to_twos_complement(velocity_wrist1, 4)
        )
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
        	self.port_handler, WRIST2_ID, ADDR_GOAL_VELOCITY, self._to_twos_complement(velocity_wrist2, 4)
        )
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, ELBOW_ID, ADDR_GOAL_VELOCITY, self._to_twos_complement(velocity_elbow, 4)
        )
        # dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
        #     self.port_handler, SHOULDER1_ID, ADDR_GOAL_VELOCITY, self._to_twos_complement(velocity_shoulder1, 4)
        # )
        # dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
        #     self.port_handler, SHOULDER2_ID, ADDR_GOAL_VELOCITY, self._to_twos_complement(velocity_shoulder2, 4)
        # )


        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Write Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            self.get_logger().error(f"Dynamixel error: {self.packet_handler.getRxPacketError(dxl_error)}")
        else:
            self.get_logger().info(f"Position Command Sent for Wrist 1: {velocity_wrist1}")
            self.get_logger().info(f"Position Command Sent for Wrist 2: {velocity_wrist2}")
            self.get_logger().info(f"Velocity Command Sent for Elbow : {velocity_elbow}")
            self.get_logger().info(f"Velocity Command Sent for Shoulder 1 : {velocity_shoulder1}")
            self.get_logger().info(f"Velocity Command Sent for Shoulder 2 : {velocity_shoulder2}")


    def _to_twos_complement(self, value, byte_size):
        max_val = 2 ** (byte_size * 8)
        return value if value >= 0 else max_val + value

def main(args=None):
    rclpy.init(args=args)
    dynamixel_node = DynamixelTeleop()
    pwm_node = gripper()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(dynamixel_node)
    executor.add_node(pwm_node)
    dynamixel_node.destroy_node()
    pwm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()