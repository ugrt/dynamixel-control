import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from numpy import interp

from dynamixel_sdk import COMM_SUCCESS
from dynamixel_sdk import PacketHandler
from dynamixel_sdk import PortHandler

from dynajoy.kinematics import ArmConfig, ArmKinematics

# ── Motor IDs ─────────────────────────────────────────────────────────────────
SHOULDER1_ID = 1
SHOULDER2_ID = 2
ELBOW_ID     = 3
WRIST1_ID    = 5
WRIST2_ID    = 4

ALL_IDS = [SHOULDER1_ID, SHOULDER2_ID, ELBOW_ID, WRIST1_ID, WRIST2_ID]

# ── Dynamixel register addresses ──────────────────────────────────────────────
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_VELOCITY    = 104
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132

# ── Operating modes ───────────────────────────────────────────────────────────
VELOCITY_CONTROL_MODE = 1
POSITION_CONTROL_MODE = 3

# ── Torque ────────────────────────────────────────────────────────────────────
TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0

# ── Velocity limits ───────────────────────────────────────────────────────────
MAX_VELOCITY = 100
MIN_VELOCITY = -100


class DynamixelTeleop(Node):
    """
    ROS 2 node for joystick-based Dynamixel arm control.

    All robot parameters are declared as ROS 2 parameters and loaded
    from config/robot_params.yaml at launch.  Inspect or change at runtime:

        ros2 param list /command_publisher
        ros2 param get  /command_publisher arm.L1
        ros2 param set  /command_publisher ik.step_mm 8.0

    Two control modes
    -----------------
    Velocity mode (default)
        Left stick Y    → wrist pitch  (both wrist motors together)
        Right stick Y   → shoulder pitch
        D-pad X         → elbow
        LB held         → wrist differential (left=rotation, right=shoulder1 only)

    IK mode  (toggle with X button)
        Right stick X   → move end-effector X (reach)
        Right stick Y   → move end-effector Y (height)
        IK solves shoulder+elbow+wrist, sends position commands.

    Other
        B button → torque reset
        X button → toggle IK / velocity mode
    """

    def __init__(self):
        super().__init__('command_publisher')

        # ── Declare ALL parameters (ROS 2 parameter server) ───────────────────
        # Values come from config/robot_params.yaml passed in the launch file.
        # Defaults here match the physical arm diagram.
        self.declare_parameter('arm.L1',          437.0)
        self.declare_parameter('arm.L2',          437.0)
        self.declare_parameter('arm.L3',          220.0)
        self.declare_parameter('arm.base_height', 123.0)

        self.declare_parameter('joint.shoulder_min', -3.1416)
        self.declare_parameter('joint.shoulder_max',  3.1416)
        self.declare_parameter('joint.elbow_min',    -1.5708)
        self.declare_parameter('joint.elbow_max',     3.1416)
        self.declare_parameter('joint.wrist_min',    -3.1416)
        self.declare_parameter('joint.wrist_max',     3.1416)

        self.declare_parameter('enc.counts_per_rev',   4096)
        self.declare_parameter('enc.position_neutral', 1750)
        self.declare_parameter('enc.min_position',        0)
        self.declare_parameter('enc.max_position',     3500)

        self.declare_parameter('ik.step_mm', 5.0)

        self.declare_parameter('device.port',     '/dev/ttyUSB0')
        self.declare_parameter('device.baudrate', 57600)
        self.declare_parameter('device.protocol', 2.0)

        # ── Build kinematics from ROS params ──────────────────────────────────
        p = self.get_parameters  # shorthand
        cfg = ArmConfig(
            L1          = self._p('arm.L1'),
            L2          = self._p('arm.L2'),
            L3          = self._p('arm.L3'),
            base_height = self._p('arm.base_height'),

            shoulder_min = self._p('joint.shoulder_min'),
            shoulder_max = self._p('joint.shoulder_max'),
            elbow_min    = self._p('joint.elbow_min'),
            elbow_max    = self._p('joint.elbow_max'),
            wrist_min    = self._p('joint.wrist_min'),
            wrist_max    = self._p('joint.wrist_max'),

            counts_per_rev   = self._p('enc.counts_per_rev'),
            position_neutral = self._p('enc.position_neutral'),
            min_position     = self._p('enc.min_position'),
            max_position     = self._p('enc.max_position'),

            ik_step_mm = self._p('ik.step_mm'),
        )
        self.kin = ArmKinematics(cfg)

        self.get_logger().info(
            f"Arm config: L1={cfg.L1}mm  L2={cfg.L2}mm  "
            f"L3={cfg.L3}mm  base={cfg.base_height}mm  step={cfg.ik_step_mm}mm"
        )

        # ── Dynamixel port setup ──────────────────────────────────────────────
        port     = self._p('device.port')
        baudrate = self._p('device.baudrate')
        protocol = self._p('device.protocol')

        self.port_handler   = PortHandler(port)
        self.packet_handler = PacketHandler(protocol)

        if not self.port_handler.openPort():
            self.get_logger().error(f"Failed to open port {port}")
            return
        if not self.port_handler.setBaudRate(baudrate):
            self.get_logger().error(f"Failed to set baudrate {baudrate}")
            return

        self._current_mode = VELOCITY_CONTROL_MODE
        self._set_torque(True)

        self.get_logger().info(
            "Dynamixel ready.\n"
            "  [Velocity mode]  Left-Y=wrist  Right-Y=shoulder  D-pad-X=elbow\n"
            "  [X button]       Toggle IK mode (Cartesian end-effector)\n"
            "  [B button]       Reset torque\n"
            "  ros2 param list /command_publisher  — view all params"
        )

        # ── IK mode state ─────────────────────────────────────────────────────
        self.ik_mode = False
        self.ik_target_x = (cfg.L1 + cfg.L2) * 0.6   # initial reach target
        self.ik_target_y = cfg.base_height              # initial height target
        self._prev_ik_button = 0

        # ── Velocity state ────────────────────────────────────────────────────
        self._vel = {mid: 0 for mid in ALL_IDS}

        # ── Joystick subscription ─────────────────────────────────────────────
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 1)

    # ── Helper: read a declared ROS parameter value ────────────────────────────
    def _p(self, name: str):
        return self.get_parameter(name).value

    # ── Torque ────────────────────────────────────────────────────────────────
    def _set_torque(self, enable: bool):
        val = TORQUE_ENABLE if enable else TORQUE_DISABLE
        for mid in ALL_IDS:
            self.packet_handler.write1ByteTxRx(
                self.port_handler, mid, ADDR_TORQUE_ENABLE, val)

    # ── Operating mode ────────────────────────────────────────────────────────
    def _set_operating_mode(self, mode: int):
        """
        Switch all motors between velocity (1) and position (3) control.
        Torque must be off while changing the operating mode register.
        """
        if mode == self._current_mode:
            return
        self._set_torque(False)
        for mid in ALL_IDS:
            self.packet_handler.write1ByteTxRx(
                self.port_handler, mid, ADDR_OPERATING_MODE, mode)
        self._set_torque(True)
        self._current_mode = mode
        label = "Position" if mode == POSITION_CONTROL_MODE else "Velocity"
        self.get_logger().info(f"Operating mode → {label} Control")

    # ── Two's complement (Dynamixel velocity encoding) ────────────────────────
    @staticmethod
    def _twos(value: int, byte_size: int = 4) -> int:
        max_val = 2 ** (byte_size * 8)
        return value if value >= 0 else max_val + value

    # ── Joystick callback ─────────────────────────────────────────────────────
    def joy_callback(self, joy_msg: Joy):
        # Axes
        y_axis  = -joy_msg.axes[1]   # Left stick U/D  (inverted: up = +)
        right_x =  joy_msg.axes[3]   # Right stick L/R
        right_y =  joy_msg.axes[4]   # Right stick U/D
        dpad_x  =  joy_msg.axes[6]   # D-pad L/R

        # Buttons
        torque_reset = joy_msg.buttons[1]   # B
        ik_button    = joy_msg.buttons[2]   # X — IK toggle
        l_bumper     = joy_msg.buttons[4]   # LB

        # ── IK mode toggle (rising edge) ──────────────────────────────────────
        if ik_button and not self._prev_ik_button:
            self.ik_mode = not self.ik_mode
            if self.ik_mode:
                # Seed IK target from current motor positions via FK
                pos_s, _, _ = self.packet_handler.read4ByteTxRx(
                    self.port_handler, SHOULDER1_ID, ADDR_PRESENT_POSITION)
                pos_e, _, _ = self.packet_handler.read4ByteTxRx(
                    self.port_handler, ELBOW_ID, ADDR_PRESENT_POSITION)
                pos_w, _, _ = self.packet_handler.read4ByteTxRx(
                    self.port_handler, WRIST1_ID, ADDR_PRESENT_POSITION)
                th1, th2, th3 = self.kin.positions_to_joint_angles(pos_s, pos_e, pos_w)
                x0, y0, _, _  = self.kin.forward_kinematics(th1, th2, th3)
                self.ik_target_x = x0
                self.ik_target_y = y0
                self._set_operating_mode(POSITION_CONTROL_MODE)
                self.get_logger().info(
                    f"IK mode ON  — target: ({x0:.1f}, {y0:.1f}) mm  "
                    f"[right stick to move, X to exit]")
            else:
                self._set_operating_mode(VELOCITY_CONTROL_MODE)
                self.get_logger().info("IK mode OFF — velocity mode restored")
        self._prev_ik_button = ik_button

        # ── Torque reset ──────────────────────────────────────────────────────
        if torque_reset:
            self._set_torque(False)
            self._set_torque(True)
            self.get_logger().info("Torque reset")
            return

        # ── Dispatch ──────────────────────────────────────────────────────────
        if self.ik_mode:
            self._ik_control(right_x, right_y)
        else:
            self._velocity_control(y_axis, right_y, dpad_x, l_bumper)

    # ── IK control mode ───────────────────────────────────────────────────────
    def _ik_control(self, right_x: float, right_y: float):
        """Move end-effector target with right stick, solve IK, send positions."""
        step = self.kin.cfg.ik_step_mm
        new_x = self.ik_target_x + right_x * step
        new_y = self.ik_target_y + right_y * step

        result = self.kin.inverse_kinematics(new_x, new_y)
        if result is None:
            self.get_logger().warn(
                f"IK unreachable: ({new_x:.1f}, {new_y:.1f}) mm — holding")
            return

        self.ik_target_x = new_x
        self.ik_target_y = new_y

        theta1, theta2, theta3 = result
        ps, pe, pw = self.kin.joint_angles_to_positions(theta1, theta2, theta3)

        err = self.kin.validate_ik(theta1, theta2, theta3, new_x, new_y)

        # Shoulder (parallel — both motors same position)
        self.packet_handler.write4ByteTxRx(
            self.port_handler, SHOULDER1_ID, ADDR_GOAL_POSITION, ps)
        self.packet_handler.write4ByteTxRx(
            self.port_handler, SHOULDER2_ID, ADDR_GOAL_POSITION, ps)
        # Elbow
        self.packet_handler.write4ByteTxRx(
            self.port_handler, ELBOW_ID, ADDR_GOAL_POSITION, pe)
        # Wrist (parallel — both motors same position)
        self.packet_handler.write4ByteTxRx(
            self.port_handler, WRIST1_ID, ADDR_GOAL_POSITION, pw)
        self.packet_handler.write4ByteTxRx(
            self.port_handler, WRIST2_ID, ADDR_GOAL_POSITION, pw)

        self.get_logger().info(
            f"IK → ({new_x:.1f}, {new_y:.1f}) mm | "
            f"θ=({theta1:.3f}, {theta2:.3f}, {theta3:.3f}) rad | "
            f"pos=({ps}, {pe}, {pw}) | err={err:.4f} mm")

    # ── Velocity control mode (original behaviour) ────────────────────────────
    def _velocity_control(
        self,
        y_axis: float,
        right_y: float,
        dpad_x: float,
        l_bumper: int,
    ):
        vel_elbow = int(interp(dpad_x, [-1, 1], [MIN_VELOCITY, MAX_VELOCITY]))

        if bool(l_bumper):
            # Differential wrist: left stick → rotation (motors oppose each other)
            # Right stick → SHOULDER1 only (independent)
            vel_w1 =  int(interp(y_axis,  [-1, 1], [MIN_VELOCITY, MAX_VELOCITY]))
            vel_w2 = -vel_w1
            vel_s1 =  int(interp(right_y, [-1, 1], [MIN_VELOCITY, MAX_VELOCITY]))
            vel_s2 = 0
        else:
            # Normal: both wrist motors same direction (pitch), both shoulders same
            vel_w1 = int(interp(y_axis,  [-1, 1], [MIN_VELOCITY, MAX_VELOCITY]))
            vel_w2 = vel_w1
            vel_s1 = int(interp(right_y, [-1, 1], [MIN_VELOCITY, MAX_VELOCITY]))
            vel_s2 = vel_s1

        # Write velocities
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, WRIST1_ID, ADDR_GOAL_VELOCITY, self._twos(vel_w1))
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, WRIST2_ID, ADDR_GOAL_VELOCITY, self._twos(vel_w2))
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, ELBOW_ID, ADDR_GOAL_VELOCITY, self._twos(vel_elbow))
        # Shoulder velocity — uncomment when ready:
        # self.packet_handler.write4ByteTxRx(
        #     self.port_handler, SHOULDER1_ID, ADDR_GOAL_VELOCITY, self._twos(vel_s1))
        # self.packet_handler.write4ByteTxRx(
        #     self.port_handler, SHOULDER2_ID, ADDR_GOAL_VELOCITY, self._twos(vel_s2))

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(
                f"Write error: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            self.get_logger().error(
                f"Dynamixel error: {self.packet_handler.getRxPacketError(dxl_error)}")
        else:
            self.get_logger().info(
                f"Vel — W1:{vel_w1:+4d}  W2:{vel_w2:+4d}  "
                f"Elbow:{vel_elbow:+4d}  S1:{vel_s1:+4d}  S2:{vel_s2:+4d}")


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
