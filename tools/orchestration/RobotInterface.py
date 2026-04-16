"""
OP08#-1 Harmonic Reducer Motor Assembly Cell — Robot Interface

Abstraction layer for FANUC and ROKAE robot arms.  Provides a unified
API for motion, tool control, vision-guided picking, and force-controlled
placement used by the CellOrchestrator.
"""

from __future__ import annotations

import abc
import logging
import time
from dataclasses import dataclass, field
from typing import Optional

from src.data_types.cell_types import (
    CalibrationMode,
    JointAngles,
    MotionType,
    Pose,
    RobotID,
    ToolType,
    VisionOffset,
)

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Coordinate frame descriptors
# ---------------------------------------------------------------------------

@dataclass
class CoordinateFrame:
    """Named coordinate frame relative to the world origin."""
    name: str
    origin: Pose
    parent: str = "world"


# ---------------------------------------------------------------------------
# Waypoint container
# ---------------------------------------------------------------------------

@dataclass
class Waypoint:
    """Taught waypoint with metadata."""
    name: str
    pose: Pose
    joints: Optional[JointAngles] = None
    frame: str = "world"
    speed_mm_s: float = 100.0
    blend_radius_mm: float = 0.0
    comment: str = ""


# ---------------------------------------------------------------------------
# Abstract base class
# ---------------------------------------------------------------------------

class RobotInterface(abc.ABC):
    """Unified robot control interface for FANUC / ROKAE arms."""

    def __init__(self, robot_id: RobotID, ip_address: str,
                 specs: dict) -> None:
        self.robot_id = robot_id
        self.ip_address = ip_address
        self.specs = specs
        self._connected: bool = False
        self._servo_on: bool = False
        self._current_tool: Optional[ToolType] = None
        self._frames: dict[str, CoordinateFrame] = {}
        self._waypoints: dict[str, Waypoint] = {}
        self._force_threshold_n: float = 10.0
        self._torque_threshold_nm: float = 0.5
        logger.info("RobotInterface created for %s at %s",
                     robot_id.value, ip_address)

    # --- Connection lifecycle -----------------------------------------

    @abc.abstractmethod
    def connect(self) -> bool:
        """Open communication channel to the robot controller."""

    @abc.abstractmethod
    def disconnect(self) -> None:
        """Release the communication channel."""

    @abc.abstractmethod
    def servo_on(self) -> bool:
        """Enable servo drives."""

    @abc.abstractmethod
    def servo_off(self) -> None:
        """Disable servo drives gracefully."""

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_servo_on(self) -> bool:
        return self._servo_on

    # --- Motion commands ----------------------------------------------

    @abc.abstractmethod
    def move_j(self, target: JointAngles | Pose, speed_pct: float = 50.0,
               accel_pct: float = 50.0, blend_radius_mm: float = 0.0
               ) -> bool:
        """Joint-interpolated motion to *target*."""

    @abc.abstractmethod
    def move_l(self, target: Pose, speed_mm_s: float = 100.0,
               accel_mm_s2: float = 500.0, blend_radius_mm: float = 0.0
               ) -> bool:
        """Linear (Cartesian) motion to *target*."""

    @abc.abstractmethod
    def move_c(self, via: Pose, target: Pose, speed_mm_s: float = 50.0
               ) -> bool:
        """Circular motion through *via* to *target*."""

    def move(self, motion_type: MotionType, target: Pose | JointAngles,
             **kwargs) -> bool:
        """Dispatch to the correct motion primitive."""
        dispatch = {
            MotionType.MOVEJ: self.move_j,
            MotionType.MOVEL: self.move_l,
        }
        handler = dispatch.get(motion_type)
        if handler is None:
            raise ValueError(f"Unsupported single-target motion: {motion_type}")
        return handler(target, **kwargs)

    @abc.abstractmethod
    def stop(self) -> None:
        """Immediate controlled stop (decelerate to zero)."""

    @abc.abstractmethod
    def get_current_pose(self) -> Pose:
        """Return the current TCP pose in the active frame."""

    @abc.abstractmethod
    def get_current_joints(self) -> JointAngles:
        """Return current joint angles in degrees."""

    @abc.abstractmethod
    def is_motion_complete(self) -> bool:
        """Return True when the last commanded motion is finished."""

    def wait_motion_complete(self, timeout_s: float = 30.0) -> bool:
        """Block until the motion completes or timeout elapses."""
        t0 = time.monotonic()
        while not self.is_motion_complete():
            if time.monotonic() - t0 > timeout_s:
                logger.error("%s: motion timeout after %.1f s",
                             self.robot_id.value, timeout_s)
                return False
            time.sleep(0.01)
        return True

    # --- Tool control -------------------------------------------------

    @abc.abstractmethod
    def gripper_open(self) -> bool:
        """Open the pneumatic / electric gripper."""

    @abc.abstractmethod
    def gripper_close(self) -> bool:
        """Close the gripper and confirm part presence."""

    @abc.abstractmethod
    def suction_on(self) -> bool:
        """Activate vacuum suction cup and verify vacuum level."""

    @abc.abstractmethod
    def suction_off(self) -> bool:
        """Release vacuum."""

    @abc.abstractmethod
    def screwdriver_engage(self, torque_nm: float, speed_rpm: float
                           ) -> bool:
        """Run the screw driver spindle at the given parameters."""

    @abc.abstractmethod
    def screwdriver_stop(self) -> None:
        """Stop the screw driver spindle immediately."""

    @abc.abstractmethod
    def get_tool_state(self) -> dict:
        """Return a dict with current tool I/O states."""

    def set_active_tool(self, tool: ToolType) -> None:
        """Select which tool is logically active (for TCP switching)."""
        self._current_tool = tool
        logger.info("%s: active tool set to %s",
                     self.robot_id.value, tool.value)

    # --- Vision-guided picking ----------------------------------------

    @abc.abstractmethod
    def trigger_vision(self) -> bool:
        """Send a trigger pulse to the vision system."""

    @abc.abstractmethod
    def get_vision_offset(self) -> VisionOffset:
        """Retrieve the latest vision offset (dX, dY, dRz)."""

    def apply_correction(self, nominal: Pose,
                         offset: VisionOffset) -> Pose:
        """Apply a 2-D vision correction to a nominal pick pose."""
        import math
        cos_rz = math.cos(math.radians(offset.drz))
        sin_rz = math.sin(math.radians(offset.drz))
        corrected_x = nominal.x + offset.dx * cos_rz - offset.dy * sin_rz
        corrected_y = nominal.y + offset.dx * sin_rz + offset.dy * cos_rz
        return Pose(
            x=corrected_x,
            y=corrected_y,
            z=nominal.z,
            rx=nominal.rx,
            ry=nominal.ry,
            rz=nominal.rz + offset.drz,
        )

    def vision_guided_pick(self, nominal_pose: Pose,
                           approach_offset_z: float = 50.0) -> bool:
        """Full sequence: trigger vision -> correct -> approach -> pick."""
        if not self.trigger_vision():
            logger.error("%s: vision trigger failed", self.robot_id.value)
            return False

        offset = self.get_vision_offset()
        if offset.confidence < 0.80:
            logger.warning("%s: low vision confidence %.2f",
                           self.robot_id.value, offset.confidence)
            return False

        corrected = self.apply_correction(nominal_pose, offset)
        approach = Pose(corrected.x, corrected.y,
                        corrected.z + approach_offset_z,
                        corrected.rx, corrected.ry, corrected.rz)

        # Approach above the part
        if not self.move_l(approach, speed_mm_s=200.0):
            return False
        self.wait_motion_complete()

        # Descend to pick
        if not self.move_l(corrected, speed_mm_s=30.0):
            return False
        self.wait_motion_complete()

        # Grasp
        ok = (self.gripper_close()
              if self._current_tool == ToolType.GRIPPER
              else self.suction_on())

        # Retract
        self.move_l(approach, speed_mm_s=100.0)
        self.wait_motion_complete()
        return ok

    # --- Force-controlled placement -----------------------------------

    @abc.abstractmethod
    def get_tcp_force(self) -> tuple[float, float, float]:
        """Return (Fx, Fy, Fz) at the TCP in Newtons."""

    @abc.abstractmethod
    def get_tcp_torque(self) -> tuple[float, float, float]:
        """Return (Tx, Ty, Tz) at the TCP in Nm."""

    def force_controlled_place(self, target: Pose,
                               force_limit_n: float = 10.0,
                               approach_speed_mm_s: float = 5.0) -> bool:
        """Descend toward *target* until force threshold is reached."""
        self._force_threshold_n = force_limit_n
        step_mm = 0.5
        current = self.get_current_pose()
        while current.z > target.z:
            next_pose = Pose(current.x, current.y,
                             current.z - step_mm,
                             current.rx, current.ry, current.rz)
            self.move_l(next_pose, speed_mm_s=approach_speed_mm_s)
            self.wait_motion_complete()
            fz = self.get_tcp_force()[2]
            if abs(fz) >= force_limit_n:
                logger.info("%s: contact detected at Fz=%.2f N",
                            self.robot_id.value, fz)
                break
            current = self.get_current_pose()

        # Release
        ok = (self.gripper_open()
              if self._current_tool == ToolType.GRIPPER
              else self.suction_off())
        # Retract
        retract = Pose(current.x, current.y, current.z + 30.0,
                       current.rx, current.ry, current.rz)
        self.move_l(retract, speed_mm_s=50.0)
        self.wait_motion_complete()
        return ok

    # --- Coordinate frame management ----------------------------------

    def register_frame(self, frame: CoordinateFrame) -> None:
        self._frames[frame.name] = frame
        logger.debug("%s: frame '%s' registered",
                     self.robot_id.value, frame.name)

    def get_frame(self, name: str) -> CoordinateFrame:
        return self._frames[name]

    # --- Waypoint teaching interface ----------------------------------

    def teach_waypoint(self, name: str, comment: str = "") -> Waypoint:
        """Record the current robot pose as a named waypoint."""
        pose = self.get_current_pose()
        joints = self.get_current_joints()
        wp = Waypoint(name=name, pose=pose, joints=joints, comment=comment)
        self._waypoints[name] = wp
        logger.info("%s: waypoint '%s' taught", self.robot_id.value, name)
        return wp

    def get_waypoint(self, name: str) -> Waypoint:
        return self._waypoints[name]

    def move_to_waypoint(self, name: str,
                         motion_type: MotionType = MotionType.MOVEJ,
                         **kwargs) -> bool:
        """Move to a previously taught waypoint."""
        wp = self._waypoints[name]
        return self.move(motion_type, wp.pose, **kwargs)

    # --- Homing -------------------------------------------------------

    def home(self) -> bool:
        """Move to the home (ready) joint configuration."""
        home_deg = self.specs.get("home_joints_deg")
        if home_deg is None:
            logger.error("%s: no home position defined",
                         self.robot_id.value)
            return False
        home_joints = JointAngles(*home_deg[:6])
        return self.move_j(home_joints, speed_pct=30.0)
