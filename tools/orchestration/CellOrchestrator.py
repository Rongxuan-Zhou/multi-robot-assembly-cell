"""
OP08#-1 Harmonic Reducer Motor Assembly Cell — Cell Orchestrator

Master controller for the three-robot assembly workstation at CloudMinds
Robotics.  Manages cell state machine, zone-based parallel execution,
turntable synchronisation, product-variant recipes, takt-time monitoring,
OEE calculation, and the EtherCAT master interface.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

from src.data_types.cell_types import (
    ActuatorVariant,
    AssemblyStep,
    CellState,
    Pose,
    RobotID,
    ScrewProfile,
    StationID,
    TaktRecord,
    ZoneID,
    ZoneState,
)
from src.global_variables.system_config import (
    ETHERCAT_CYCLE_TIME_US,
    ETHERCAT_MASTER_INTERFACE,
    ETHERCAT_SLAVES,
    OEE_TARGET,
    TAKT_TIME_S,
    TAKT_WARNING_THRESHOLD,
    TURNTABLE_CLAMP_TIME_MS,
    TURNTABLE_INDEX_DEG,
    TURNTABLE_POSITION_TOLERANCE_DEG,
    TURNTABLE_SETTLING_TIME_MS,
    VARIANT_STEP_COUNTS,
)
from src.robot_control.MaterialHandler import MaterialHandler
from src.robot_control.RobotInterface import RobotInterface
from src.robot_control.ScrewTighteningManager import ScrewTighteningManager
from src.robot_control.VisionSystem import VisionSystem

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Recipe management
# ---------------------------------------------------------------------------

@dataclass
class AssemblyRecipe:
    """Complete recipe for one actuator variant."""
    variant: ActuatorVariant
    zone1_steps: list[AssemblyStep] = field(default_factory=list)
    zone2_steps: list[AssemblyStep] = field(default_factory=list)
    zone3_steps: list[AssemblyStep] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Zone executor
# ---------------------------------------------------------------------------

class ZoneExecutor:
    """Runs all assembly steps for one zone in its own thread."""

    def __init__(self, zone_id: ZoneID, robot: RobotInterface) -> None:
        self.zone_id = zone_id
        self.robot = robot
        self._state: ZoneState = ZoneState.IDLE
        self._steps: list[AssemblyStep] = []
        self._current_step_idx: int = 0
        self._elapsed_ms: int = 0
        self._thread: Optional[threading.Thread] = None
        self._abort_event = threading.Event()
        self._step_callback: Optional[Callable[[ZoneID, AssemblyStep], None]] = None

    @property
    def state(self) -> ZoneState:
        return self._state

    @property
    def elapsed_ms(self) -> int:
        return self._elapsed_ms

    def load_steps(self, steps: list[AssemblyStep]) -> None:
        self._steps = list(steps)
        self._current_step_idx = 0
        self._state = ZoneState.IDLE

    def set_step_callback(self, cb: Callable[[ZoneID, AssemblyStep], None]
                          ) -> None:
        self._step_callback = cb

    def start(self) -> None:
        """Begin executing the loaded step list asynchronously."""
        if not self._steps:
            logger.info("Zone %s: no steps — marking DONE", self.zone_id.name)
            self._state = ZoneState.DONE
            return
        self._abort_event.clear()
        self._state = ZoneState.BUSY
        self._thread = threading.Thread(target=self._run,
                                        name=f"Zone-{self.zone_id.name}",
                                        daemon=True)
        self._thread.start()

    def abort(self) -> None:
        self._abort_event.set()
        self._state = ZoneState.ERROR

    def wait(self, timeout_s: float = 120.0) -> bool:
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)
        return self._state == ZoneState.DONE

    # ---- internal ----------------------------------------------------

    def _run(self) -> None:
        t0 = time.monotonic()
        try:
            for idx, step in enumerate(self._steps):
                if self._abort_event.is_set():
                    self._state = ZoneState.ERROR
                    return
                self._current_step_idx = idx
                logger.info("Zone %s step %d/%d: %s",
                            self.zone_id.name, idx + 1, len(self._steps),
                            step.description)
                if self._step_callback:
                    self._step_callback(self.zone_id, step)
                # Simulate step execution time
                time.sleep(step.duration_ms / 1000.0)
            self._state = ZoneState.DONE
        except Exception:
            logger.exception("Zone %s execution failed", self.zone_id.name)
            self._state = ZoneState.ERROR
        finally:
            self._elapsed_ms = int((time.monotonic() - t0) * 1000)


# ---------------------------------------------------------------------------
# EtherCAT master wrapper
# ---------------------------------------------------------------------------

class EtherCATMaster:
    """Thin wrapper around the IgH EtherCAT master (pybind11 bindings)."""

    def __init__(self, interface: str = ETHERCAT_MASTER_INTERFACE,
                 cycle_us: int = ETHERCAT_CYCLE_TIME_US) -> None:
        self.interface = interface
        self.cycle_us = cycle_us
        self._active: bool = False

    def activate(self) -> bool:
        """Initialise the master and bring all slaves to OP state."""
        logger.info("EtherCAT master activating on %s (cycle %d us)",
                     self.interface, self.cycle_us)
        # Production: igh_master.activate()
        for slave in ETHERCAT_SLAVES:
            logger.debug("  Slave %d: %s — %s",
                         slave.position, slave.name, slave.description)
        self._active = True
        return True

    def deactivate(self) -> None:
        self._active = False
        logger.info("EtherCAT master deactivated")

    @property
    def is_active(self) -> bool:
        return self._active

    def read_pdo(self, slave_pos: int, index: int,
                 subindex: int) -> int:
        """Read a process-data object from a slave."""
        return 0

    def write_pdo(self, slave_pos: int, index: int,
                  subindex: int, value: int) -> None:
        """Write a process-data object to a slave."""

    def get_slave_state(self, slave_pos: int) -> str:
        return "OP"


# ---------------------------------------------------------------------------
# Turntable controller
# ---------------------------------------------------------------------------

class TurntableController:
    """120 degree indexing turntable with pneumatic clamps."""

    def __init__(self, ethercat: EtherCATMaster, servo_slave_pos: int = 6
                 ) -> None:
        self._ec = ethercat
        self._slave_pos = servo_slave_pos
        self._current_deg: float = 0.0
        self._clamped: bool = False

    @property
    def position_deg(self) -> float:
        return self._current_deg

    @property
    def is_clamped(self) -> bool:
        return self._clamped

    def clamp(self) -> bool:
        self._clamped = True
        time.sleep(TURNTABLE_CLAMP_TIME_MS / 1000.0)
        logger.debug("Turntable clamped")
        return True

    def unclamp(self) -> bool:
        self._clamped = False
        time.sleep(TURNTABLE_CLAMP_TIME_MS / 1000.0)
        logger.debug("Turntable unclamped")
        return True

    def index_120(self) -> bool:
        """Rotate 120 degrees and wait for settling."""
        if self._clamped:
            logger.error("Cannot rotate — turntable is clamped")
            return False
        target = (self._current_deg + TURNTABLE_INDEX_DEG) % 360.0
        logger.info("Turntable indexing: %.1f → %.1f deg",
                     self._current_deg, target)
        # Production: command servo via EtherCAT, wait for position
        rotation_time_s = TURNTABLE_INDEX_DEG / 60.0  # 60 deg/s
        time.sleep(rotation_time_s)
        self._current_deg = target
        time.sleep(TURNTABLE_SETTLING_TIME_MS / 1000.0)
        error = abs(self._current_deg - target)
        if error > TURNTABLE_POSITION_TOLERANCE_DEG:
            logger.error("Turntable position error %.4f > %.4f deg",
                         error, TURNTABLE_POSITION_TOLERANCE_DEG)
            return False
        return True

    def home(self) -> bool:
        """Rotate to 0 degree reference."""
        self._current_deg = 0.0
        logger.info("Turntable homed to 0 deg")
        return True


# ---------------------------------------------------------------------------
# OEE calculator
# ---------------------------------------------------------------------------

class OEECalculator:
    """Simple OEE = Availability x Performance x Quality tracker."""

    def __init__(self, takt_target_s: float = TAKT_TIME_S) -> None:
        self.takt_target_s = takt_target_s
        self._records: list[TaktRecord] = []
        self._total_planned_s: float = 0.0
        self._total_downtime_s: float = 0.0
        self._total_good: int = 0
        self._total_bad: int = 0

    def record_cycle(self, record: TaktRecord) -> None:
        self._records.append(record)

    def add_downtime(self, seconds: float) -> None:
        self._total_downtime_s += seconds

    def add_planned_time(self, seconds: float) -> None:
        self._total_planned_s += seconds

    def mark_good(self) -> None:
        self._total_good += 1

    def mark_bad(self) -> None:
        self._total_bad += 1

    @property
    def availability(self) -> float:
        if self._total_planned_s == 0:
            return 1.0
        return ((self._total_planned_s - self._total_downtime_s)
                / self._total_planned_s)

    @property
    def performance(self) -> float:
        if not self._records:
            return 1.0
        ideal_total_s = len(self._records) * self.takt_target_s
        actual_total_s = sum(r.total_ms / 1000.0 for r in self._records)
        if actual_total_s == 0:
            return 1.0
        return min(ideal_total_s / actual_total_s, 1.0)

    @property
    def quality(self) -> float:
        total = self._total_good + self._total_bad
        if total == 0:
            return 1.0
        return self._total_good / total

    @property
    def oee(self) -> float:
        return self.availability * self.performance * self.quality

    def summary(self) -> dict:
        return {
            "availability": round(self.availability, 4),
            "performance": round(self.performance, 4),
            "quality": round(self.quality, 4),
            "oee": round(self.oee, 4),
            "target": OEE_TARGET,
            "cycles": len(self._records),
        }


# ---------------------------------------------------------------------------
# Master cell orchestrator
# ---------------------------------------------------------------------------

class CellOrchestrator:
    """Top-level controller coordinating all robots, turntable, vision,
    screw stations, material handling, and production tracking."""

    def __init__(
        self,
        robots: dict[RobotID, RobotInterface],
        vision_systems: dict[str, VisionSystem],
        screw_managers: dict[RobotID, ScrewTighteningManager],
        material_handler: MaterialHandler,
    ) -> None:
        self._robots = robots
        self._vision = vision_systems
        self._screw = screw_managers
        self._material = material_handler

        # EtherCAT
        self._ec = EtherCATMaster()
        self._turntable = TurntableController(self._ec)

        # State machine
        self._state: CellState = CellState.IDLE
        self._prev_state: CellState = CellState.IDLE
        self._error_msg: str = ""

        # Zone executors
        self._zones: dict[ZoneID, ZoneExecutor] = {
            ZoneID.ZONE1: ZoneExecutor(ZoneID.ZONE1,
                                       robots[RobotID.R1_FANUC]),
            ZoneID.ZONE2: ZoneExecutor(ZoneID.ZONE2,
                                       robots[RobotID.R2_ROKAE]),
            ZoneID.ZONE3: ZoneExecutor(ZoneID.ZONE3,
                                       robots[RobotID.R3_ROKAE]),
        }

        # Recipe management
        self._recipes: dict[ActuatorVariant, AssemblyRecipe] = {}
        self._active_variant: ActuatorVariant = ActuatorVariant.ACT_8

        # Production tracking
        self._cycle_id: int = 0
        self._oee = OEECalculator()
        self._cycle_lock = threading.Lock()

        logger.info("CellOrchestrator initialised")

    # === Properties ====================================================

    @property
    def state(self) -> CellState:
        return self._state

    @property
    def active_variant(self) -> ActuatorVariant:
        return self._active_variant

    @property
    def cycle_count(self) -> int:
        return self._cycle_id

    # === State-machine transitions =====================================

    def _set_state(self, new_state: CellState) -> None:
        self._prev_state = self._state
        self._state = new_state
        logger.info("Cell state: %s → %s",
                     self._prev_state.value, new_state.value)

    def request_start(self) -> bool:
        """Operator presses START."""
        if self._state == CellState.READY:
            self._set_state(CellState.RUNNING)
            return True
        logger.warning("Cannot start from state %s", self._state.value)
        return False

    def request_pause(self) -> None:
        if self._state == CellState.RUNNING:
            self._set_state(CellState.PAUSED)

    def request_resume(self) -> None:
        if self._state == CellState.PAUSED:
            self._set_state(CellState.RUNNING)

    def request_stop(self) -> None:
        for zone in self._zones.values():
            zone.abort()
        self._set_state(CellState.IDLE)

    def acknowledge_error(self) -> None:
        if self._state == CellState.ERROR:
            self._error_msg = ""
            self._set_state(CellState.IDLE)

    def _enter_error(self, msg: str) -> None:
        self._error_msg = msg
        logger.error("CELL ERROR: %s", msg)
        self._set_state(CellState.ERROR)

    # === Initialisation ================================================

    def initialise(self) -> bool:
        """Power-on sequence: EtherCAT → robots → vision → homing."""
        self._set_state(CellState.HOMING)

        # 1. Activate EtherCAT
        if not self._ec.activate():
            self._enter_error("EtherCAT activation failed")
            return False

        # 2. Connect robots
        for rid, robot in self._robots.items():
            if not robot.connect():
                self._enter_error(f"Robot {rid.value} connection failed")
                return False
            if not robot.servo_on():
                self._enter_error(f"Robot {rid.value} servo-on failed")
                return False

        # 3. Connect vision systems
        for cam_id, cam in self._vision.items():
            if not cam.connect():
                self._enter_error(f"Camera {cam_id} connection failed")
                return False

        # 4. Home all robots
        for rid, robot in self._robots.items():
            if not robot.home():
                self._enter_error(f"Robot {rid.value} homing failed")
                return False

        # 5. Home turntable
        if not self._turntable.home():
            self._enter_error("Turntable homing failed")
            return False
        self._turntable.clamp()

        self._set_state(CellState.READY)
        return True

    def shutdown(self) -> None:
        """Graceful shutdown sequence."""
        logger.info("Cell shutdown initiated")
        for zone in self._zones.values():
            zone.abort()
        for robot in self._robots.values():
            robot.servo_off()
            robot.disconnect()
        for cam in self._vision.values():
            cam.disconnect()
        self._turntable.unclamp()
        self._ec.deactivate()
        self._set_state(CellState.IDLE)

    # === Recipe management =============================================

    def register_recipe(self, recipe: AssemblyRecipe) -> None:
        self._recipes[recipe.variant] = recipe
        logger.info("Recipe registered for variant %s",
                     recipe.variant.name)

    def select_variant(self, variant: ActuatorVariant) -> bool:
        """Switch to a different actuator variant (from HMI)."""
        if variant not in self._recipes:
            logger.error("No recipe for variant %s", variant.name)
            return False
        self._active_variant = variant
        logger.info("Active variant set to %s", variant.name)
        return True

    def _load_recipe_to_zones(self) -> None:
        recipe = self._recipes.get(self._active_variant)
        if recipe is None:
            raise RuntimeError(f"No recipe for {self._active_variant.name}")
        self._zones[ZoneID.ZONE1].load_steps(recipe.zone1_steps)
        self._zones[ZoneID.ZONE2].load_steps(recipe.zone2_steps)
        if self._active_variant.requires_brake_assembly:
            self._zones[ZoneID.ZONE3].load_steps(recipe.zone3_steps)
        else:
            self._zones[ZoneID.ZONE3].load_steps([])
            logger.info("Zone3 skipped for variant %s",
                         self._active_variant.name)

    # === Main production cycle =========================================

    def run_cycle(self) -> Optional[TaktRecord]:
        """Execute one complete production cycle (all three zones in
        parallel, then turntable index)."""
        if self._state != CellState.RUNNING:
            logger.warning("Cannot run cycle in state %s",
                           self._state.value)
            return None

        with self._cycle_lock:
            self._cycle_id += 1
            cycle_id = self._cycle_id

        t_cycle_start = time.monotonic()
        logger.info("=== Cycle %d START (variant %s) ===",
                     cycle_id, self._active_variant.name)

        # Load recipe steps
        self._load_recipe_to_zones()

        # --- Phase 1: parallel zone execution -------------------------
        for zone in self._zones.values():
            zone.start()

        # Wait for all zones to complete
        all_ok = True
        for zone in self._zones.values():
            if not zone.wait(timeout_s=TAKT_TIME_S * 1.5):
                self._enter_error(
                    f"Zone {zone.zone_id.name} failed or timed out")
                all_ok = False

        if not all_ok:
            return None

        # Collect zone times
        zone_times: dict[ZoneID, int] = {}
        for zid, zone in self._zones.items():
            zone_times[zid] = zone.elapsed_ms

        # --- Phase 2: turntable rotation synchronisation --------------
        # All zones must be DONE before we rotate
        zones_ready = all(z.state == ZoneState.DONE
                          for z in self._zones.values())
        if not zones_ready:
            self._enter_error("Not all zones ready for turntable index")
            return None

        self._turntable.unclamp()
        if not self._turntable.index_120():
            self._enter_error("Turntable index failed")
            return None
        self._turntable.clamp()

        # --- Phase 3: record takt & OEE ------------------------------
        total_ms = int((time.monotonic() - t_cycle_start) * 1000)
        bottleneck = max(zone_times, key=zone_times.get)  # type: ignore[arg-type]

        record = TaktRecord(
            cycle_id=cycle_id,
            variant=self._active_variant,
            zone_times_ms=zone_times,
            bottleneck=bottleneck,
            total_ms=total_ms,
        )

        self._oee.record_cycle(record)
        self._oee.mark_good()
        self._oee.add_planned_time(TAKT_TIME_S)
        record.oee = self._oee.oee

        # Takt-time monitoring
        takt_ratio = total_ms / (TAKT_TIME_S * 1000)
        if takt_ratio > 1.0:
            logger.warning("Cycle %d EXCEEDED takt: %d ms (%.0f%%)",
                           cycle_id, total_ms, takt_ratio * 100)
        elif takt_ratio > TAKT_WARNING_THRESHOLD:
            logger.warning("Cycle %d near takt limit: %d ms (%.0f%%)",
                           cycle_id, total_ms, takt_ratio * 100)
        else:
            logger.info("Cycle %d OK: %d ms (%.0f%% of takt)",
                        cycle_id, total_ms, takt_ratio * 100)

        logger.info("=== Cycle %d END — OEE %.1f%% ===",
                     cycle_id, record.oee * 100)
        return record

    # === Continuous production loop ====================================

    def run_production(self, max_cycles: int = 0) -> None:
        """Run production cycles until stopped or max_cycles reached."""
        if not self.request_start():
            return
        cycles_done = 0
        while self._state == CellState.RUNNING:
            record = self.run_cycle()
            if record is None:
                break
            cycles_done += 1
            if 0 < max_cycles <= cycles_done:
                logger.info("Reached max_cycles=%d — stopping",
                            max_cycles)
                break
            # Report to MES periodically
            if cycles_done % 10 == 0:
                self._material.report_to_mes("http://mes:8080/api/parts")

        self._set_state(CellState.READY)

    # === Status / diagnostics ==========================================

    def get_status(self) -> dict[str, Any]:
        return {
            "cell_state": self._state.value,
            "variant": self._active_variant.name,
            "cycle_id": self._cycle_id,
            "turntable_deg": self._turntable.position_deg,
            "turntable_clamped": self._turntable.is_clamped,
            "ethercat_active": self._ec.is_active,
            "zones": {
                zid.name: zone.state.value
                for zid, zone in self._zones.items()
            },
            "oee": self._oee.summary(),
            "material_buffer": self._material.buffer_status(),
            "error": self._error_msg,
        }

    def get_screw_statistics(self) -> dict[str, Any]:
        return {
            rid.value: mgr.get_spc_summary()
            for rid, mgr in self._screw.items()
        }
