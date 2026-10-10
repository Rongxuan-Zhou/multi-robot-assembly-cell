"""
OP08#-1 Harmonic Reducer Motor Assembly Cell — System Configuration

Centralised constants and default parameters for the CloudMinds Robotics
multi-robot assembly workstation.  Values here are loaded at startup and
can be overridden by YAML configs in ``config/``.
"""

from __future__ import annotations

from tools.orchestration.cell_types import (
    ActuatorVariant,
    EtherCATSlave,
    RobotID,
    SafetyZone,
    ScrewProfile,
    ZoneID,
)


# ===================================================================
# Takt-time & production targets
# ===================================================================

TAKT_TIME_S: float = 94.0                 # Target cycle time (seconds)
TAKT_WARNING_THRESHOLD: float = 0.90      # Warn when cycle > 90 % of takt
OEE_TARGET: float = 0.85                  # Overall equipment effectiveness

# ===================================================================
# Turntable specifications
# ===================================================================

TURNTABLE_DIAMETER_MM: float = 1200.0
TURNTABLE_INDEX_POSITIONS: int = 3        # 120 degree indexing
TURNTABLE_INDEX_DEG: float = 120.0
TURNTABLE_SETTLING_TIME_MS: int = 500     # Vibration settle after index
TURNTABLE_CLAMP_TIME_MS: int = 200        # Pneumatic clamp engage time
TURNTABLE_ROTATION_SPEED_DEG_S: float = 60.0
TURNTABLE_POSITION_TOLERANCE_DEG: float = 0.02

# ===================================================================
# Robot specifications
# ===================================================================

ROBOT_SPECS: dict[RobotID, dict] = {
    RobotID.R1_FANUC: {
        "brand": "FANUC",
        "model": "M-10iD/12",
        "payload_kg": 12.0,
        "reach_mm": 1441.0,
        "repeatability_mm": 0.04,
        "dof": 6,
        "ip_address": "192.168.1.11",
        "comm_protocol": "FANUC_ROBOGUIDE",
        "tools": ["gripper", "suction", "vision"],
        "home_joints_deg": [0.0, -30.0, 0.0, 0.0, -60.0, 0.0],
    },
    # 6-axis ROKAE per source equipment list; model is representative, source names no model
    RobotID.R2_ROKAE: {
        "brand": "ROKAE",
        "model": "XB7L",
        "payload_kg": 7.0,
        "reach_mm": 906.0,
        "repeatability_mm": 0.03,
        "dof": 6,
        "ip_address": "192.168.1.12",
        "comm_protocol": "ROKAE_SDK",
        "tools": ["gripper", "screwdriver"],
        "home_joints_deg": [0.0, -45.0, 0.0, 0.0, -45.0, 0.0],
    },
    # 6-axis ROKAE per source equipment list; model is representative, source names no model
    RobotID.R3_ROKAE: {
        "brand": "ROKAE",
        "model": "XB7",
        "payload_kg": 7.0,
        "reach_mm": 707.0,
        "repeatability_mm": 0.02,
        "dof": 6,
        "ip_address": "192.168.1.13",
        "comm_protocol": "ROKAE_SDK",
        "tools": ["gripper", "screwdriver"],
        "home_joints_deg": [0.0, -45.0, 0.0, 0.0, -45.0, 0.0],
    },
}

# ===================================================================
# Screw tightening profiles
# ===================================================================

# Torque targets are design assumptions; the source gives no torque values.
SCREW_PROFILES: dict[ScrewProfile, dict] = {
    ScrewProfile.M25x4: {
        "target_torque_nm": 0.3,
        "torque_tolerance_nm": 0.05,
        "target_angle_deg": 720.0,
        "angle_tolerance_deg": 90.0,
        "approach_speed_rpm": 300,
        "snug_speed_rpm": 150,
        "final_speed_rpm": 50,
        "verify_reverse_deg": 5.0,
        "max_retries": 2,
        "screw_length_mm": 4.0,
        "thread_pitch_mm": 0.45,
    },
    ScrewProfile.M25x6: {
        "target_torque_nm": 0.5,
        "torque_tolerance_nm": 0.08,
        "target_angle_deg": 1080.0,
        "angle_tolerance_deg": 120.0,
        "approach_speed_rpm": 300,
        "snug_speed_rpm": 120,
        "final_speed_rpm": 40,
        "verify_reverse_deg": 5.0,
        "max_retries": 2,
        "screw_length_mm": 6.0,
        "thread_pitch_mm": 0.45,
    },
}

# Motor-mount screw per variant (source page I / page VI BOM:
# #8 M2.5x4 x4, #11&#18 M2.5x6 x4).
VARIANT_MOTOR_SCREW: dict[ActuatorVariant, ScrewProfile] = {
    ActuatorVariant.ACT_8: ScrewProfile.M25x4,
    ActuatorVariant.ACT_11: ScrewProfile.M25x6,
    ActuatorVariant.ACT_18: ScrewProfile.M25x6,
}

BRAKE_BASE_SCREW: ScrewProfile = ScrewProfile.M25x6   # x2, #11&#18 only

# ===================================================================
# EtherCAT slave addresses (IgH EtherCAT Master)
# ===================================================================

ETHERCAT_MASTER_INTERFACE: str = "enp3s0"
ETHERCAT_CYCLE_TIME_US: int = 1000        # 1 ms cycle

ETHERCAT_SLAVES: list[EtherCATSlave] = [
    EtherCATSlave(0, 0, 0x00000002, 0x09252048, "Beckhoff_EK1100",
                  "EtherCAT Coupler"),
    EtherCATSlave(0, 1, 0x00000002, 0x03F63052, "Beckhoff_EL1008",
                  "8-ch Digital Input 24V"),
    EtherCATSlave(0, 2, 0x00000002, 0x07D43052, "Beckhoff_EL2008",
                  "8-ch Digital Output 24V"),
    EtherCATSlave(0, 3, 0x00000002, 0x0C1E3052, "Beckhoff_EL3102",
                  "2-ch Analog Input +/-10V"),
    EtherCATSlave(0, 4, 0x00000002, 0x10063052, "Beckhoff_EL4102",
                  "2-ch Analog Output +/-10V"),
    EtherCATSlave(0, 5, 0x00000002, 0x13ED3052, "Beckhoff_EL5101",
                  "Incremental Encoder Interface"),
    # Turntable servo drive
    EtherCATSlave(0, 6, 0x000001DD, 0x10305070, "Delta_ASDA_A3",
                  "Turntable Servo Drive"),
    # Screw driver controllers
    EtherCATSlave(0, 7, 0x00000539, 0x00000301, "Atlas_QST",
                  "Screw Controller R2"),
    EtherCATSlave(0, 8, 0x00000539, 0x00000302, "Atlas_QST",
                  "Screw Controller R3"),
    # Safety module
    EtherCATSlave(0, 9, 0x00000002, 0x17923052, "Beckhoff_EL6910",
                  "TwinSAFE Logic"),
    EtherCATSlave(0, 10, 0x00000002, 0x17803052, "Beckhoff_EL1904",
                  "4-ch Safety DI"),
    EtherCATSlave(0, 11, 0x00000002, 0x17843052, "Beckhoff_EL2904",
                  "4-ch Safety DO"),
]

# ===================================================================
# Vision camera configuration
# ===================================================================

# Source: 1 handling vision + 2 tightening vision; mounting on the screwdriver
# end-effector is a design assumption.
VISION_CAMERAS: dict[str, dict] = {
    "cam_r1_eye_in_hand": {
        "type": "GigE",
        "ip_address": "192.168.20.11",
        "model": "Basler_acA2440-35gm",
        "resolution": (2448, 2048),
        "pixel_size_um": 3.45,
        "focal_length_mm": 12.0,
        "exposure_us": 5000,
        "gain_db": 6.0,
        "calibration_mode": "eye_in_hand",
        "robot": "R1_FANUC",
    },
    "cam_r2_tightening": {
        "type": "GigE",
        "ip_address": "192.168.20.12",
        "model": "Basler_acA1920-40gm",
        "resolution": (1920, 1200),
        "pixel_size_um": 5.86,
        "focal_length_mm": 16.0,
        "exposure_us": 8000,
        "gain_db": 4.0,
        "calibration_mode": "eye_in_hand",
        "robot": "R2_ROKAE",
    },
    "cam_r3_tightening": {
        "type": "GigE",
        "ip_address": "192.168.20.13",
        "model": "Basler_acA1920-40gm",
        "resolution": (1920, 1200),
        "pixel_size_um": 5.86,
        "focal_length_mm": 16.0,
        "exposure_us": 8000,
        "gain_db": 4.0,
        "calibration_mode": "eye_in_hand",
        "robot": "R3_ROKAE",
    },
}

# ===================================================================
# Safety zone definitions (world frame, mm)
# ===================================================================

SAFETY_ZONES: list[SafetyZone] = [
    SafetyZone(ZoneID.ZONE1, x_min=-800, x_max=0, y_min=-600, y_max=600,
               z_min=0, z_max=1200, max_speed_mm_s=500.0),
    SafetyZone(ZoneID.ZONE2, x_min=0, x_max=800, y_min=-600, y_max=0,
               z_min=0, z_max=1000, max_speed_mm_s=250.0),
    SafetyZone(ZoneID.ZONE3, x_min=0, x_max=800, y_min=0, y_max=600,
               z_min=0, z_max=1000, max_speed_mm_s=250.0),
]

# ===================================================================
# Material handling — trays and stackers
# ===================================================================

TRAY_STACKER_COUNT: int = 3
TRAYS_PER_STACKER: int = 5
TRAY_COUNT_TOTAL: int = 15  # source equipment list: 15 trays across 3 box stackers (even split assumed)
TRAY_ROWS: int = 4
TRAY_COLS: int = 5
TRAY_SLOT_PITCH_MM: float = 30.0
C_RACK_POSITIONS: int = 6                 # Slots on C-shape rack

# ===================================================================
# Assembly recipes per variant
# ===================================================================

# Step numbers refer to the source sequence (page II, steps 1-17).
VARIANT_STEP_COUNTS: dict[ActuatorVariant, dict] = {
    ActuatorVariant.ACT_8: {
        "zone1_steps": 3,       # steps 1-3: unload finished, load body, load motor
        "zone2_steps": 7,       # steps 5-11: motor, cable dressing, body, clamp, screws x4, unclamp
        "zone3_steps": 1,       # step 17 only: R2 returns product to Station 3 fixture
    },
    ActuatorVariant.ACT_11: {
        "zone1_steps": 4,       # steps 1-4: + load brake magnet base
        "zone2_steps": 7,       # steps 5-11
        "zone3_steps": 6,       # steps 12-17: brake base, clamp, screws x2, unclamp, return
    },
    ActuatorVariant.ACT_18: {
        "zone1_steps": 4,       # steps 1-4
        "zone2_steps": 7,       # steps 5-11
        "zone3_steps": 6,       # steps 12-17
    },
}

# ===================================================================
# Communication ports
# ===================================================================

HMI_OPC_UA_PORT: int = 4840
MES_REST_PORT: int = 8080
CODESYS_GATEWAY_PORT: int = 11740
MODBUS_TCP_PORT: int = 502
