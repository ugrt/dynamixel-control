"""
ODrive S1 — CAN bus interface
==============================
Thin wrapper around python-can that speaks the ODrive CAN simple-serial
protocol (firmware 0.6 / S1).

To swap to USB later: subclass OdriveCANInterface and override _send(),
enable(), and disable().  The rest of the API is identical.

ODrive CAN protocol reference:
  https://docs.odriverobotics.com/v/latest/manual/can-protocol.html

Arbitration ID encoding:  (node_id << 5) | cmd_id

ODrive native units
  Position  turns          (1 turn = 2π rad)
  Velocity  turns / second
  Torque    N·m
"""
from __future__ import annotations

import math
import struct
import logging
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    pass

log = logging.getLogger(__name__)

try:
    import can as _can
    _CAN_AVAILABLE = True
except ImportError:
    _CAN_AVAILABLE = False
    log.warning(
        "python-can not installed — ODrive CAN commands will be no-ops. "
        "Install with: pip install python-can"
    )

try:
    import numpy as _np
    _NP_AVAILABLE = True
except ImportError:
    _NP_AVAILABLE = False

# ── ODrive CAN command IDs ─────────────────────────────────────────────────────
_CMD_SET_AXIS_STATE      = 0x07
_CMD_GET_ENCODER_EST     = 0x09   # RTR → node replies with pos + vel
_CMD_SET_CONTROLLER_MODE = 0x0B
_CMD_SET_INPUT_POS       = 0x0C   # float32 pos + float16 vel_ff + float16 torque_ff
_CMD_SET_INPUT_VEL       = 0x0D   # float32 vel + float32 torque_ff

# ── Axis states ────────────────────────────────────────────────────────────────
AXIS_STATE_IDLE                = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

# ── Controller modes ───────────────────────────────────────────────────────────
CONTROL_MODE_VELOCITY_CONTROL = 2
CONTROL_MODE_POSITION_CONTROL = 3

# ── Input modes ────────────────────────────────────────────────────────────────
INPUT_MODE_PASSTHROUGH = 1
INPUT_MODE_VEL_RAMP    = 2   # smooth velocity ramp
INPUT_MODE_POS_FILTER  = 3   # low-pass filtered position tracking
INPUT_MODE_TRAP_TRAJ   = 5   # trapezoidal trajectory (needs vel/accel limits set)


def _pack_float16(value: float) -> bytes:
    """Pack a Python float as IEEE 754 float16 little-endian (2 bytes)."""
    if _NP_AVAILABLE:
        return _np.float16(value).tobytes()
    # Fallback: encode as float32 truncated to 2 bytes (loses precision but safe)
    return struct.pack('<e', value)   # Python 3.6+ supports 'e' format


class OdriveCANInterface:
    """
    Manages multiple ODrive S1 axes on a single CAN bus.

    Parameters
    ----------
    channel  : SocketCAN interface name, e.g. ``"can0"``
    bitrate  : CAN bitrate in bps — must match the ODrive firmware setting
               (ODrive S1 factory default is 250 000 bps)
    node_ids : mapping ``{name: node_id}``  e.g.
               ``{"base": 0, "shoulder": 1, "elbow": 2}``

    Usage
    -----
    ::

        od = OdriveCANInterface(
            channel="can0",
            node_ids={"base": 0, "shoulder": 1, "elbow": 2}
        )
        od.enable_all()
        od.set_velocity_mode_all()

        # velocity teleoperation
        od.vel_rps("shoulder", 0.5)   # 0.5 rad/s

        # IK position targets
        od.set_position_mode_all()
        od.pos_rad("shoulder", 1.047)  # 60°

        od.close()
    """

    def __init__(
        self,
        channel: str = "can0",
        bitrate: int = 250_000,
        node_ids: dict[str, int] | None = None,
    ):
        self._node_ids: dict[str, int] = node_ids or {}
        self._bus = None

        if not _CAN_AVAILABLE:
            return

        try:
            self._bus = _can.interface.Bus(
                channel=channel,
                bustype="socketcan",
                bitrate=bitrate,
            )
            log.info("ODrive CAN bus opened: %s @ %d bps", channel, bitrate)
        except Exception as exc:
            log.error("Cannot open CAN bus %s: %s", channel, exc)

    # ── Internal helpers ───────────────────────────────────────────────────────

    def _send(self, node_id: int, cmd_id: int, data: bytes) -> None:
        """Send a CAN frame. Silent no-op if the bus is unavailable."""
        if self._bus is None:
            return
        msg = _can.Message(
            arbitration_id=(node_id << 5) | cmd_id,
            data=data,
            is_extended_id=False,
        )
        try:
            self._bus.send(msg)
        except Exception as exc:
            log.error(
                "CAN send failed (node %d, cmd 0x%02X): %s", node_id, cmd_id, exc
            )

    def node(self, name: str) -> int:
        """Return the CAN node ID for a named axis. Raises KeyError if unknown."""
        return self._node_ids[name]

    # ── Axis state ─────────────────────────────────────────────────────────────

    def set_axis_state(self, node_id: int, state: int) -> None:
        """Send Set_Axis_State (cmd 0x07). payload: uint32 LE."""
        self._send(node_id, _CMD_SET_AXIS_STATE, struct.pack('<I', state))

    def enable(self, node_id: int) -> None:
        """Put axis into CLOSED_LOOP_CONTROL (motor energised)."""
        self.set_axis_state(node_id, AXIS_STATE_CLOSED_LOOP_CONTROL)

    def disable(self, node_id: int) -> None:
        """Put axis into IDLE (motor de-energised)."""
        self.set_axis_state(node_id, AXIS_STATE_IDLE)

    def enable_all(self) -> None:
        for nid in self._node_ids.values():
            self.enable(nid)

    def disable_all(self) -> None:
        for nid in self._node_ids.values():
            self.disable(nid)

    # ── Controller / input mode ────────────────────────────────────────────────

    def set_controller_mode(
        self,
        node_id: int,
        control_mode: int,
        input_mode: int,
    ) -> None:
        """
        Send Set_Controller_Mode (cmd 0x0B).
        payload: control_mode (uint32) + input_mode (uint32), both LE.
        """
        data = struct.pack('<II', control_mode, input_mode)
        self._send(node_id, _CMD_SET_CONTROLLER_MODE, data)

    def set_velocity_mode(self, node_id: int) -> None:
        """Switch axis to velocity + vel-ramp input mode."""
        self.set_controller_mode(
            node_id,
            CONTROL_MODE_VELOCITY_CONTROL,
            INPUT_MODE_VEL_RAMP,
        )

    def set_position_mode(self, node_id: int) -> None:
        """Switch axis to position + trapezoidal trajectory mode."""
        self.set_controller_mode(
            node_id,
            CONTROL_MODE_POSITION_CONTROL,
            INPUT_MODE_TRAP_TRAJ,
        )

    def set_velocity_mode_all(self) -> None:
        for nid in self._node_ids.values():
            self.set_velocity_mode(nid)

    def set_position_mode_all(self) -> None:
        for nid in self._node_ids.values():
            self.set_position_mode(nid)

    # ── Position control ───────────────────────────────────────────────────────

    def set_position_turns(
        self,
        node_id: int,
        position_turns: float,
        vel_ff: float = 0.0,
        torque_ff: float = 0.0,
    ) -> None:
        """
        Send Set_Input_Pos (cmd 0x0C).
        payload: pos (float32) + vel_ff (float16) + torque_ff (float16) = 8 bytes
        """
        data = (
            struct.pack('<f', position_turns)
            + _pack_float16(vel_ff)
            + _pack_float16(torque_ff)
        )
        self._send(node_id, _CMD_SET_INPUT_POS, data)

    def set_position_rad(
        self,
        node_id: int,
        angle_rad: float,
        vel_ff: float = 0.0,
        torque_ff: float = 0.0,
    ) -> None:
        """Convenience: accepts radians, converts to turns (1 turn = 2π rad)."""
        self.set_position_turns(
            node_id,
            angle_rad / (2.0 * math.pi),
            vel_ff,
            torque_ff,
        )

    # ── Velocity control ───────────────────────────────────────────────────────

    def set_velocity_turns(
        self,
        node_id: int,
        vel_turns_per_sec: float,
        torque_ff: float = 0.0,
    ) -> None:
        """
        Send Set_Input_Vel (cmd 0x0D).
        payload: vel (float32) + torque_ff (float32) = 8 bytes
        """
        data = struct.pack('<ff', vel_turns_per_sec, torque_ff)
        self._send(node_id, _CMD_SET_INPUT_VEL, data)

    def set_velocity_rad_per_sec(
        self,
        node_id: int,
        vel_rps: float,
        torque_ff: float = 0.0,
    ) -> None:
        """Convenience: accepts rad/s, converts to turns/s."""
        self.set_velocity_turns(
            node_id,
            vel_rps / (2.0 * math.pi),
            torque_ff,
        )

    # ── Named-axis convenience methods ─────────────────────────────────────────

    def pos_rad(self, name: str, angle_rad: float, **kw) -> None:
        """Send position command (rad) to a named axis."""
        self.set_position_rad(self.node(name), angle_rad, **kw)

    def vel_rps(self, name: str, vel_rad_per_sec: float, **kw) -> None:
        """Send velocity command (rad/s) to a named axis."""
        self.set_velocity_rad_per_sec(self.node(name), vel_rad_per_sec, **kw)

    # ── Cleanup ────────────────────────────────────────────────────────────────

    def close(self) -> None:
        if self._bus is not None:
            self._bus.shutdown()
            log.info("ODrive CAN bus closed")
