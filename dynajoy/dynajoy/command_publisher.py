"""
DynamixelTeleop — ROS 2 node for 6-DOF joystick arm control
=============================================================
Hardware split
  ODrive S1 (CAN)  — base rotation, shoulder, elbow
  Dynamixel XM540  — wrist pitch (W270-R), wrist spin (W270-R), gripper (W150-R)

Joystick mapping
-----------------
Velocity mode (default)
  Left  stick  X   → wrist spin
  Left  stick  Y   → wrist pitch
  Right stick  X   → base rotation
  Right stick  Y   → shoulder
  D-pad        Y   → elbow
  Right trigger    → gripper close
  Left  trigger    → gripper open

IK mode (toggle with X button)
  Right stick  X   → rotate base (change arm direction)
  Right stick  Y   → reach in / out
  D-pad        Y   → height up / down
  Left  stick  Y   → wrist orientation (phi_arm in arm plane)
  Left  stick  X   → wrist spin (independent)
  Right trigger    → gripper close
  Left  trigger    → gripper open

Buttons
  B            → disable all motors (emergency stop) then re-enable
  X            → toggle IK / velocity mode
  Y            → re-enable ODrive axes (CLOSED_LOOP_CONTROL)

Parameters (all in config/robot_params.yaml)
  ros2 param list /command_publisher
  ros2 param set  /command_publisher ik.step_mm 8.0
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from numpy import interp

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

from dynajoy.kinematics import ArmConfig, ArmKinematics
from dynajoy.odrive_can import OdriveCANInterface

# ── Dynamixel XM540 motor IDs ─────────────────────────────────────────────────
# Set these IDs in the Dynamixel Wizard to match.
WRIST_PITCH_ID = 1   # XM540-W270-R  (wrist up / down)
WRIST_SPIN_ID  = 2   # XM540-W270-R  (spin wrist)
GRIPPER_ID     = 3   # XM540-W150-R  (gripper open / close)

DXL_IDS = [WRIST_PITCH_ID, WRIST_SPIN_ID, GRIPPER_ID]

# ── Dynamixel register addresses (Protocol 2.0 / XM540) ──────────────────────
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_VELOCITY    = 104
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132

VELOCITY_CONTROL_MODE = 1
POSITION_CONTROL_MODE = 3
TORQUE_ENABLE         = 1
TORQUE_DISABLE        = 0

# ── Dynamixel velocity limits (raw units) ────────────────────────────────────
DXL_MAX_VEL = 100
DXL_MIN_VEL = -100

# ── ODrive CAN node IDs ───────────────────────────────────────────────────────
BASE_NODE     = 0
SHOULDER_NODE = 1
ELBOW_NODE    = 2

ODRIVE_NAMES  = ("base", "shoulder", "elbow")   # must match node IDs above


class DynamixelTeleop(Node):
    """
    ROS 2 node — joystick-driven 6-DOF arm controller.

    Velocity mode
      ODrive joints are driven via incremental position steps
      (avoids controller-mode switching on every tick).
      Dynamixel joints use native velocity control.

    IK mode
      Cylindrical target (r, z, θ₀) is adjusted by the right stick / D-pad.
      IK solution drives ODrive + Dynamixel wrist-pitch to position targets.
      Wrist spin and gripper remain independently controlled.
    """

    def __init__(self):
        super().__init__('command_publisher')

        # ── Declare ALL ROS 2 parameters ──────────────────────────────────────
        # Values are read from config/robot_params.yaml at launch.

        # Arm geometry
        self.declare_parameter('arm.L1',          437.0)
        self.declare_parameter('arm.L2',          437.0)
        self.declare_parameter('arm.L3',          220.0)
        self.declare_parameter('arm.base_height', 123.0)

        # ODrive joint limits (rad)
        self.declare_parameter('joint.base_min',      -3.1416)
        self.declare_parameter('joint.base_max',       3.1416)
        self.declare_parameter('joint.shoulder_min',  -1.5708)
        self.declare_parameter('joint.shoulder_max',   3.1416)
        self.declare_parameter('joint.elbow_min',     -3.1416)
        self.declare_parameter('joint.elbow_max',      3.1416)

        # Dynamixel joint limits (rad)
        self.declare_parameter('joint.wrist_pitch_min', -1.5708)
        self.declare_parameter('joint.wrist_pitch_max',  1.5708)
        self.declare_parameter('joint.wrist_spin_min',  -3.1416)
        self.declare_parameter('joint.wrist_spin_max',   3.1416)
        self.declare_parameter('joint.gripper_min',      0.0)
        self.declare_parameter('joint.gripper_max',      1.5708)

        # Dynamixel XM540 encoder
        self.declare_parameter('enc.counts_per_rev',   4096)
        self.declare_parameter('enc.position_neutral', 2048)
        self.declare_parameter('enc.min_position',        0)
        self.declare_parameter('enc.max_position',     4095)

        # IK / velocity behaviour
        self.declare_parameter('ik.step_mm',    5.0)
        self.declare_parameter('ik.step_rad',   0.05)
        self.declare_parameter('vel.step_rad',  0.03)

        # ODrive CAN
        self.declare_parameter('odrive.can_channel',     'can0')
        self.declare_parameter('odrive.can_bitrate',     250000)
        self.declare_parameter('odrive.base_node_id',    BASE_NODE)
        self.declare_parameter('odrive.shoulder_node_id', SHOULDER_NODE)
        self.declare_parameter('odrive.elbow_node_id',   ELBOW_NODE)

        # Dynamixel IDs
        self.declare_parameter('dxl.wrist_pitch_id', WRIST_PITCH_ID)
        self.declare_parameter('dxl.wrist_spin_id',  WRIST_SPIN_ID)
        self.declare_parameter('dxl.gripper_id',     GRIPPER_ID)

        # Dynamixel port
        self.declare_parameter('device.port',     '/dev/ttyUSB0')
        self.declare_parameter('device.baudrate', 57600)
        self.declare_parameter('device.protocol', 2.0)

        # ── Build ArmConfig / ArmKinematics from ROS params ───────────────────
        cfg = ArmConfig(
            L1          = self._p('arm.L1'),
            L2          = self._p('arm.L2'),
            L3          = self._p('arm.L3'),
            base_height = self._p('arm.base_height'),

            base_min     = self._p('joint.base_min'),
            base_max     = self._p('joint.base_max'),
            shoulder_min = self._p('joint.shoulder_min'),
            shoulder_max = self._p('joint.shoulder_max'),
            elbow_min    = self._p('joint.elbow_min'),
            elbow_max    = self._p('joint.elbow_max'),

            wrist_pitch_min = self._p('joint.wrist_pitch_min'),
            wrist_pitch_max = self._p('joint.wrist_pitch_max'),
            wrist_spin_min  = self._p('joint.wrist_spin_min'),
            wrist_spin_max  = self._p('joint.wrist_spin_max'),
            gripper_min     = self._p('joint.gripper_min'),
            gripper_max     = self._p('joint.gripper_max'),

            dxl_counts_per_rev   = self._p('enc.counts_per_rev'),
            dxl_position_neutral = self._p('enc.position_neutral'),
            dxl_min_position     = self._p('enc.min_position'),
            dxl_max_position     = self._p('enc.max_position'),

            ik_step_mm  = self._p('ik.step_mm'),
            ik_step_rad = self._p('ik.step_rad'),
            vel_step_rad = self._p('vel.step_rad'),
        )
        self.kin = ArmKinematics(cfg)

        # ── Resolve motor IDs from params ─────────────────────────────────────
        self._dxl_ids = {
            'wrist_pitch': self._p('dxl.wrist_pitch_id'),
            'wrist_spin':  self._p('dxl.wrist_spin_id'),
            'gripper':     self._p('dxl.gripper_id'),
        }
        odrive_nodes = {
            'base':     self._p('odrive.base_node_id'),
            'shoulder': self._p('odrive.shoulder_node_id'),
            'elbow':    self._p('odrive.elbow_node_id'),
        }

        # ── ODrive CAN interface ──────────────────────────────────────────────
        self._odrive = OdriveCANInterface(
            channel  = self._p('odrive.can_channel'),
            bitrate  = self._p('odrive.can_bitrate'),
            node_ids = odrive_nodes,
        )
        self._odrive.enable_all()
        self._odrive.set_position_mode_all()   # use incremental position for both modes

        # Tracked ODrive angles (dead-reckoning from commands, rad)
        self._od_ang = {'base': 0.0, 'shoulder': 0.0, 'elbow': 0.0}
        # Send zero as the starting target
        for name, rad in self._od_ang.items():
            self._odrive.pos_rad(name, rad)

        # Tracked Dynamixel angles (rad)
        self._dxl_ang = {'wrist_pitch': 0.0, 'wrist_spin': 0.0, 'gripper': 0.0}

        # ── Dynamixel port ────────────────────────────────────────────────────
        port     = self._p('device.port')
        baudrate = self._p('device.baudrate')
        protocol = self._p('device.protocol')

        self.port_handler   = PortHandler(port)
        self.packet_handler = PacketHandler(protocol)

        if not self.port_handler.openPort():
            self.get_logger().error(f"Failed to open Dynamixel port {port}")
            return
        if not self.port_handler.setBaudRate(baudrate):
            self.get_logger().error(f"Failed to set Dynamixel baudrate {baudrate}")
            return

        self._dxl_mode = POSITION_CONTROL_MODE
        self._dxl_set_mode(POSITION_CONTROL_MODE)
        self._dxl_torque(True)

        # ── IK mode state ─────────────────────────────────────────────────────
        self.ik_mode       = False
        self._ik_r         = cfg.L1 * 0.6            # cylindrical reach (mm)
        self._ik_z         = cfg.base_height + 100.0  # world-frame height (mm)
        self._ik_theta0    = 0.0                       # base angle (rad)
        self._ik_phi_arm   = 0.0                       # wrist orientation in plane (rad)

        # ── Button debounce ───────────────────────────────────────────────────
        self._prev_ik_btn     = 0
        self._prev_enable_btn = 0

        # ── Joystick subscription ─────────────────────────────────────────────
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 1)

        self.get_logger().info(
            "\n"
            "  6-DOF arm ready\n"
            "  ODrive  : base={base}  shoulder={shoulder}  elbow={elbow} (CAN)\n"
            "  Dynamixel: wrist_pitch={wp}  wrist_spin={ws}  gripper={gr}\n"
            "\n"
            "  [Velocity]  Right-X=base  Right-Y=shoulder  D-pad-Y=elbow\n"
            "              Left-X=wrist_spin  Left-Y=wrist_pitch  Triggers=gripper\n"
            "  [X button]  Toggle IK mode\n"
            "  [B button]  Emergency disable → re-enable\n"
            "  [Y button]  Re-enable ODrive (after error)\n".format(
                **odrive_nodes,
                wp=self._dxl_ids['wrist_pitch'],
                ws=self._dxl_ids['wrist_spin'],
                gr=self._dxl_ids['gripper'],
            )
        )

    # ── Helper: read a ROS 2 parameter value ──────────────────────────────────
    def _p(self, name: str):
        return self.get_parameter(name).value

    # ── Dynamixel helpers ─────────────────────────────────────────────────────

    def _dxl_torque(self, enable: bool):
        val = TORQUE_ENABLE if enable else TORQUE_DISABLE
        for mid in self._dxl_ids.values():
            self.packet_handler.write1ByteTxRx(
                self.port_handler, mid, ADDR_TORQUE_ENABLE, val)

    def _dxl_set_mode(self, mode: int):
        if mode == self._dxl_mode:
            return
        self._dxl_torque(False)
        for mid in self._dxl_ids.values():
            self.packet_handler.write1ByteTxRx(
                self.port_handler, mid, ADDR_OPERATING_MODE, mode)
        self._dxl_torque(True)
        self._dxl_mode = mode

    @staticmethod
    def _twos(value: int, byte_size: int = 4) -> int:
        """Two's complement for Dynamixel velocity encoding."""
        max_val = 2 ** (byte_size * 8)
        return value if value >= 0 else max_val + value

    def _dxl_write_pos(self, joint: str, angle_rad: float):
        """Write a position target to a Dynamixel joint (clamped)."""
        mid   = self._dxl_ids[joint]
        count = self.kin.dxl_rad_to_count(angle_rad)
        self.packet_handler.write4ByteTxRx(
            self.port_handler, mid, ADDR_GOAL_POSITION, count)
        self._dxl_ang[joint] = angle_rad

    def _dxl_write_vel(self, joint: str, vel_raw: int):
        """Write a velocity command to a Dynamixel joint."""
        mid = self._dxl_ids[joint]
        self.packet_handler.write4ByteTxRx(
            self.port_handler, mid, ADDR_GOAL_VELOCITY, self._twos(vel_raw))

    # ── ODrive helpers ────────────────────────────────────────────────────────

    def _od_step(self, joint: str, delta_rad: float):
        """
        Increment an ODrive joint angle, clamp to limits, and send position.

        Joint limits by name:
          base     → (base_min,     base_max)
          shoulder → (shoulder_min, shoulder_max)
          elbow    → (elbow_min,    elbow_max)
        """
        cfg = self.kin.cfg
        limits = {
            'base':     (cfg.base_min,     cfg.base_max),
            'shoulder': (cfg.shoulder_min, cfg.shoulder_max),
            'elbow':    (cfg.elbow_min,    cfg.elbow_max),
        }
        lo, hi = limits[joint]
        new_ang = max(lo, min(hi, self._od_ang[joint] + delta_rad))
        self._od_ang[joint] = new_ang
        self._odrive.pos_rad(joint, new_ang)

    def _od_send_pos(self, joint: str, angle_rad: float):
        """Send a specific position (rad) to an ODrive joint, tracking state."""
        cfg = self.kin.cfg
        limits = {
            'base':     (cfg.base_min,     cfg.base_max),
            'shoulder': (cfg.shoulder_min, cfg.shoulder_max),
            'elbow':    (cfg.elbow_min,    cfg.elbow_max),
        }
        lo, hi = limits[joint]
        clamped = max(lo, min(hi, angle_rad))
        self._od_ang[joint] = clamped
        self._odrive.pos_rad(joint, clamped)

    # ── Joystick callback ─────────────────────────────────────────────────────

    def joy_callback(self, joy_msg: Joy):
        axes    = joy_msg.axes
        buttons = joy_msg.buttons

        # Axes (Xbox/similar layout)
        left_x  =  axes[0]   # Left stick  L/R  → wrist spin
        left_y  = -axes[1]   # Left stick  U/D  → wrist pitch  (inverted: up = +)
        lt      =  axes[2]   # Left  trigger    → gripper open  (1=idle, −1=full)
        right_x =  axes[3]   # Right stick L/R  → base / IK base rotate
        right_y = -axes[4]   # Right stick U/D  → shoulder / IK reach  (inverted)
        rt      =  axes[5]   # Right trigger    → gripper close (1=idle, −1=full)
        dpad_y  =  axes[7]   # D-pad U/D        → elbow / IK height

        # Buttons
        b_btn      = buttons[1]   # B — emergency disable
        x_btn      = buttons[2]   # X — toggle IK mode
        y_btn      = buttons[3]   # Y — re-enable ODrive

        # ── Emergency disable / re-enable (B, rising edge) ────────────────────
        if b_btn:
            self._odrive.disable_all()
            self._dxl_torque(False)
            self.get_logger().warn("Motors DISABLED (B button)")
            import time; time.sleep(0.5)
            self._odrive.enable_all()
            self._dxl_torque(True)
            self.get_logger().info("Motors re-enabled")
            return

        # ── ODrive re-enable (Y, rising edge) ────────────────────────────────
        if y_btn and not self._prev_enable_btn:
            self._odrive.enable_all()
            self.get_logger().info("ODrive re-enabled (Y button)")
        self._prev_enable_btn = y_btn

        # ── IK mode toggle (X, rising edge) ──────────────────────────────────
        if x_btn and not self._prev_ik_btn:
            self.ik_mode = not self.ik_mode
            if self.ik_mode:
                # Seed cylindrical IK target from current ODrive angles via FK
                x0, y0, z0, phi0 = self.kin.forward_kinematics_3d(
                    self._od_ang['base'],
                    self._od_ang['shoulder'],
                    self._od_ang['elbow'],
                    self._dxl_ang['wrist_pitch'],
                )
                self._ik_r       = math.hypot(x0, y0)
                self._ik_z       = z0
                self._ik_theta0  = self._od_ang['base']
                self._ik_phi_arm = phi0
                # Switch Dynamixel to position mode for IK
                self._dxl_set_mode(POSITION_CONTROL_MODE)
                self.get_logger().info(
                    f"IK mode ON  r={self._ik_r:.1f}mm  "
                    f"z={self._ik_z:.1f}mm  θ₀={math.degrees(self._ik_theta0):.1f}°  "
                    f"[right-stick / d-pad to move, X to exit]"
                )
            else:
                # Back to velocity (Dynamixel stays in position mode for incremental)
                self.get_logger().info("IK mode OFF — velocity mode")
        self._prev_ik_btn = x_btn

        # ── Gripper (triggers, both modes) ───────────────────────────────────
        # RT pressed → close (positive angle), LT pressed → open (toward 0)
        grip_cmd = (lt - rt) / 2.0   # −1 = close, +1 = open
        grip_step = grip_cmd * self.kin.cfg.ik_step_rad * 2
        cfg = self.kin.cfg
        new_grip = max(cfg.gripper_min,
                       min(cfg.gripper_max,
                           self._dxl_ang['gripper'] - grip_step))
        self._dxl_write_pos('gripper', new_grip)

        # ── Dispatch to control mode ──────────────────────────────────────────
        if self.ik_mode:
            self._ik_control(right_x, right_y, dpad_y, left_x, left_y)
        else:
            self._velocity_control(left_x, left_y, right_x, right_y, dpad_y)

    # ── IK control mode ───────────────────────────────────────────────────────

    def _ik_control(
        self,
        right_x: float,
        right_y: float,
        dpad_y: float,
        left_x: float,
        left_y: float,
    ):
        """
        Cylindrical Cartesian control.

        right_x → rotate base  (Δθ₀)
        right_y → reach in/out (Δr)
        dpad_y  → height       (Δz)
        left_y  → wrist orientation in arm plane (Δφ_arm)
        left_x  → wrist spin   (Δθ₄, independent)
        """
        cfg  = self.kin.cfg
        step = cfg.ik_step_mm
        rads = cfg.ik_step_rad

        # Update cylindrical targets
        self._ik_theta0  += right_x * rads
        self._ik_r       += right_y * step
        self._ik_z       += dpad_y  * step
        self._ik_phi_arm += left_y  * rads

        # Clamp theta0 to base limits
        self._ik_theta0 = max(cfg.base_min, min(cfg.base_max, self._ik_theta0))

        # Convert cylindrical to Cartesian
        x_t = self._ik_r * math.cos(self._ik_theta0)
        y_t = self._ik_r * math.sin(self._ik_theta0)
        z_t = self._ik_z

        # Solve IK
        result = self.kin.inverse_kinematics_3d(
            x_t, y_t, z_t,
            phi_arm=self._ik_phi_arm,
        )
        if result is None:
            # Roll back the deltas that caused the failure
            self._ik_theta0  -= right_x * rads
            self._ik_r       -= right_y * step
            self._ik_z       -= dpad_y  * step
            self._ik_phi_arm -= left_y  * rads
            self.get_logger().warn(
                f"IK unreachable: r={self._ik_r:.1f}  z={self._ik_z:.1f}  "
                f"θ₀={math.degrees(self._ik_theta0):.1f}° — holding"
            )
            return

        theta0, theta1, theta2, theta3 = result

        # Send ODrive joints
        self._od_send_pos('base',     theta0)
        self._od_send_pos('shoulder', theta1)
        self._od_send_pos('elbow',    theta2)

        # Wrist pitch — driven by IK to maintain phi_arm
        self._dxl_write_pos('wrist_pitch', theta3)

        # Wrist spin — independent left-stick control
        new_ws = max(
            cfg.wrist_spin_min,
            min(cfg.wrist_spin_max,
                self._dxl_ang['wrist_spin'] + left_x * rads)
        )
        self._dxl_write_pos('wrist_spin', new_ws)

        err = self.kin.validate_ik_3d(theta0, theta1, theta2, theta3, x_t, y_t, z_t)
        self.get_logger().info(
            f"IK → ({x_t:.0f},{y_t:.0f},{z_t:.0f})mm | "
            f"θ=({math.degrees(theta0):.1f}°, {math.degrees(theta1):.1f}°, "
            f"{math.degrees(theta2):.1f}°, {math.degrees(theta3):.1f}°) | "
            f"err={err:.2f}mm"
        )

    # ── Velocity / incremental-position control ───────────────────────────────

    def _velocity_control(
        self,
        left_x: float,
        left_y: float,
        right_x: float,
        right_y: float,
        dpad_y: float,
    ):
        """
        Incremental-position control for ODrive joints;
        native velocity control for Dynamixel joints.

        ODrive step size is cfg.vel_step_rad per callback at full deflection.
        Dynamixel velocity is ±DXL_MAX_VEL raw units.
        """
        cfg  = self.kin.cfg
        step = cfg.vel_step_rad

        # ── ODrive: incremental position steps ────────────────────────────────
        self._od_step('base',     right_x * step)
        self._od_step('shoulder', right_y * step)
        self._od_step('elbow',    dpad_y  * step)

        # ── Dynamixel: velocity control ───────────────────────────────────────
        self._dxl_set_mode(VELOCITY_CONTROL_MODE)

        vel_wp = int(interp(left_y,  [-1, 1], [DXL_MIN_VEL, DXL_MAX_VEL]))
        vel_ws = int(interp(left_x,  [-1, 1], [DXL_MIN_VEL, DXL_MAX_VEL]))

        result_wp, err_wp = self.packet_handler.write4ByteTxRx(
            self.port_handler,
            self._dxl_ids['wrist_pitch'],
            ADDR_GOAL_VELOCITY,
            self._twos(vel_wp),
        )
        result_ws, err_ws = self.packet_handler.write4ByteTxRx(
            self.port_handler,
            self._dxl_ids['wrist_spin'],
            ADDR_GOAL_VELOCITY,
            self._twos(vel_ws),
        )

        if result_wp != COMM_SUCCESS or result_ws != COMM_SUCCESS:
            self.get_logger().error("Dynamixel write error")
        else:
            self.get_logger().debug(
                f"Vel — base:{self._od_ang['base']:.3f}rad  "
                f"sh:{self._od_ang['shoulder']:.3f}rad  "
                f"el:{self._od_ang['elbow']:.3f}rad  "
                f"wp:{vel_wp:+d}  ws:{vel_ws:+d}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelTeleop()
    try:
        rclpy.spin(node)
    finally:
        node._odrive.disable_all()
        node._dxl_torque(False)
        node._odrive.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
