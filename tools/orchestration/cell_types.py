"""
OP08#-1 Harmonic Reducer Motor Assembly Cell — Data Types

Enumerations and dataclasses used across all modules in the
CloudMinds Robotics multi-robot assembly cell.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional


# ---------------------------------------------------------------------------
# Enumerations
# ---------------------------------------------------------------------------

class ActuatorVariant(enum.Enum):
    """Product variants assembled on this workstation."""
    ACT_8 = 8       # Standard — R3 steps skipped
    ACT_11 = 11     # Brake-equipped — R3 performs brake magnet base assembly
    ACT_18 = 18     # Brake-equipped — R3 performs brake magnet base assembly

    @property
    def requires_brake_assembly(self) -> bool:
        return self in (ActuatorVariant.ACT_11, ActuatorVariant.ACT_18)


class RobotID(enum.Enum):
    """Physical robot identifiers."""
    R1_FANUC = "R1"   # FANUC — material handling
    R2_ROKAE = "R2"   # ROKAE — motor mount + motor + 4 screws
    R3_ROKAE = "R3"   # ROKAE — brake magnet base + 2 screws


class ZoneID(enum.Enum):
    """Safety / execution zones (120 degree turntable sectors)."""
    ZONE1 = 1   # R1 material handling zone
    ZONE2 = 2   # R2 main assembly zone
    ZONE3 = 3   # R3 brake magnet base zone


class StationID(enum.Enum):
    """Turntable fixture stations (physical nests on the table)."""
    STATION_A = "A"
    STATION_B = "B"
    STATION_C = "C"


class CellState(enum.Enum):
    """Top-level cell state machine states."""
    IDLE = "IDLE"
    HOMING = "HOMING"
    READY = "READY"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    ERROR = "ERROR"
    ESTOP = "ESTOP"


class ZoneState(enum.Enum):
    """Per-zone execution states."""
    IDLE = "IDLE"
    BUSY = "BUSY"
    DONE = "DONE"
    ERROR = "ERROR"
    WAITING = "WAITING"     # Waiting for turntable rotation


class MotionType(enum.Enum):
    """Robot motion interpolation modes."""
    MOVEJ = "MoveJ"     # Joint interpolation
    MOVEL = "MoveL"     # Linear (Cartesian) interpolation
    MOVEC = "MoveC"     # Circular interpolation


class ToolType(enum.Enum):
    """End-effector tool types available on the cell."""
    GRIPPER = "gripper"
    SUCTION = "suction"
    SCREWDRIVER = "screwdriver"
    VISION = "vision"


class ScrewProfile(enum.Enum):
    """Screw tightening profile identifiers."""
    M25x4 = "M2.5x4"   # Motor mount screws (R2)
    M25x6 = "M2.5x6"   # Brake magnet base screws (R3)


class TrayType(enum.Enum):
    """Tray types managed by material handling."""
    MOTOR_MOUNT = "motor_mount"
    MOTOR = "motor"
    BRAKE_MAGNET_BASE = "brake_magnet_base"
    SCREW_M25x4 = "screw_m25x4"
    SCREW_M25x6 = "screw_m25x6"


class CalibrationMode(enum.Enum):
    """Hand-eye calibration configurations."""
    EYE_IN_HAND = "eye_in_hand"       # Camera on robot flange (R1)
    EYE_TO_HAND = "eye_to_hand"       # Camera fixed on worktable


# ---------------------------------------------------------------------------
# Dataclasses
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Pose:
    """6-DOF Cartesian pose (mm / deg)."""
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float


@dataclass(frozen=True)
class JointAngles:
    """Robot joint configuration (degrees)."""
    j1: float
    j2: float
    j3: float
    j4: float
    j5: float
    j6: float


@dataclass
class VisionOffset:
    """2-D vision correction applied before pick/place."""
    dx: float = 0.0        # mm
    dy: float = 0.0        # mm
    drz: float = 0.0       # deg
    confidence: float = 0.0
    timestamp: datetime = field(default_factory=datetime.now)


@dataclass
class AssemblyStep:
    """Single step within a product assembly recipe."""
    step_id: int
    zone: ZoneID
    robot: RobotID
    description: str
    duration_ms: int
    requires_vision: bool = False
    requires_force: bool = False
    screw_profile: Optional[ScrewProfile] = None
    screw_count: int = 0


@dataclass
class ScrewResult:
    """Outcome of a single screw tightening operation."""
    screw_index: int
    torque_nm: float
    angle_deg: float
    ok: bool
    timestamp: datetime = field(default_factory=datetime.now)
    duration_ms: int = 0
    retry_count: int = 0


@dataclass
class TaktRecord:
    """Production cycle timing record."""
    cycle_id: int
    variant: ActuatorVariant
    zone_times_ms: dict[ZoneID, int] = field(default_factory=dict)
    bottleneck: Optional[ZoneID] = None
    total_ms: int = 0
    oee: float = 0.0
    timestamp: datetime = field(default_factory=datetime.now)


@dataclass
class TraySlot:
    """Occupancy information for a single tray slot."""
    row: int
    col: int
    occupied: bool = False
    part_type: Optional[TrayType] = None
    barcode: Optional[str] = None


@dataclass
class EtherCATSlave:
    """Descriptor for one EtherCAT slave on the fieldbus."""
    alias: int
    position: int
    vendor_id: int
    product_code: int
    name: str
    description: str = ""


@dataclass
class SafetyZone:
    """Rectangular safety zone definition in world frame (mm)."""
    zone_id: ZoneID
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float
    max_speed_mm_s: float = 250.0
