"""
OP08#-1 Harmonic Reducer Motor Assembly Cell — Screw Tightening Manager

Controls the multi-step screw-driving process for M2.5x4 (R2, motor
mount) and M2.5x6 (R3, brake magnet base) fasteners.  Tracks torque /
angle curves, computes Cpk, and manages the screw feeder and bit wear.
"""

from __future__ import annotations

import logging
import math
import statistics
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from src.data_types.cell_types import ScrewProfile, ScrewResult
from src.global_variables.system_config import SCREW_PROFILES

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Tightening phase enum
# ---------------------------------------------------------------------------

class TightenPhase(Enum):
    IDLE = "idle"
    APPROACH = "approach"
    SNUG = "snug"
    FINAL = "final"
    VERIFY = "verify"
    DONE = "done"
    ERROR = "error"


# ---------------------------------------------------------------------------
# Feeder state
# ---------------------------------------------------------------------------

@dataclass
class FeederState:
    """State of one pneumatic screw feeder."""
    feeder_id: str
    profile: ScrewProfile
    capacity: int = 500
    remaining: int = 500
    bit_cycles: int = 0
    bit_max_cycles: int = 5000
    feeder_ready: bool = True
    bit_present: bool = True

    @property
    def refill_needed(self) -> bool:
        return self.remaining <= 10

    @property
    def bit_change_needed(self) -> bool:
        return self.bit_cycles >= self.bit_max_cycles


# ---------------------------------------------------------------------------
# SPC statistics tracker
# ---------------------------------------------------------------------------

@dataclass
class SPCTracker:
    """Tracks torque and angle values for Cpk computation."""
    profile: ScrewProfile
    torque_values: list[float] = field(default_factory=list)
    angle_values: list[float] = field(default_factory=list)
    _window_size: int = 50

    def record(self, torque: float, angle: float) -> None:
        self.torque_values.append(torque)
        self.angle_values.append(angle)
        if len(self.torque_values) > self._window_size:
            self.torque_values = self.torque_values[-self._window_size:]
            self.angle_values = self.angle_values[-self._window_size:]

    def compute_cpk(self, target: float, tolerance: float
                    ) -> Optional[float]:
        """Compute process capability index Cpk for a list of values."""
        values = self.torque_values
        if len(values) < 10:
            return None
        mean = statistics.mean(values)
        sigma = statistics.stdev(values)
        if sigma == 0:
            return None
        usl = target + tolerance
        lsl = target - tolerance
        cpu = (usl - mean) / (3 * sigma)
        cpl = (mean - lsl) / (3 * sigma)
        return min(cpu, cpl)

    @property
    def torque_cpk(self) -> Optional[float]:
        spec = SCREW_PROFILES[self.profile]
        return self.compute_cpk(spec["target_torque_nm"],
                                spec["torque_tolerance_nm"])

    @property
    def angle_cpk(self) -> Optional[float]:
        spec = SCREW_PROFILES[self.profile]
        return self.compute_cpk(spec["target_angle_deg"],
                                spec["angle_tolerance_deg"])


# ---------------------------------------------------------------------------
# Main manager class
# ---------------------------------------------------------------------------

class ScrewTighteningManager:
    """Orchestrates multi-step screw tightening for one robot/station."""

    def __init__(self, profile: ScrewProfile, feeder_id: str) -> None:
        self.profile = profile
        self.spec = SCREW_PROFILES[profile]
        self.feeder = FeederState(feeder_id=feeder_id, profile=profile)
        self.spc = SPCTracker(profile=profile)
        self._phase: TightenPhase = TightenPhase.IDLE
        self._results: list[ScrewResult] = []
        self._total_ok: int = 0
        self._total_nok: int = 0
        logger.info("ScrewTighteningManager created for %s (feeder=%s)",
                     profile.value, feeder_id)

    # --- Feeder management -------------------------------------------

    def check_feeder_ready(self) -> bool:
        """Verify feeder has screws and bit is present."""
        if self.feeder.refill_needed:
            logger.warning("Feeder '%s' needs refill (%d remaining)",
                           self.feeder.feeder_id, self.feeder.remaining)
            return False
        if not self.feeder.bit_present:
            logger.error("Feeder '%s' bit not present",
                          self.feeder.feeder_id)
            return False
        if not self.feeder.feeder_ready:
            logger.warning("Feeder '%s' not ready",
                           self.feeder.feeder_id)
            return False
        return True

    def consume_screw(self) -> None:
        """Decrement the screw counter after a successful pick."""
        self.feeder.remaining = max(0, self.feeder.remaining - 1)
        self.feeder.bit_cycles += 1

    def refill_feeder(self, count: int) -> None:
        """Operator has refilled the feeder."""
        self.feeder.remaining = min(self.feeder.capacity,
                                    self.feeder.remaining + count)
        logger.info("Feeder '%s' refilled to %d",
                     self.feeder.feeder_id, self.feeder.remaining)

    def replace_bit(self) -> None:
        """Operator replaced the driver bit."""
        self.feeder.bit_cycles = 0
        self.feeder.bit_present = True
        logger.info("Bit replaced on feeder '%s'", self.feeder.feeder_id)

    # --- Auto bit-change sequence ------------------------------------

    def auto_bit_change_sequence(self) -> bool:
        """Automated bit exchange when wear limit is reached.

        Returns True if the bit was replaced (or not needed).
        """
        if not self.feeder.bit_change_needed:
            return True
        logger.warning("Bit wear limit reached on '%s' (%d cycles). "
                       "Starting auto-change.",
                       self.feeder.feeder_id, self.feeder.bit_cycles)
        # In production: robot moves to bit magazine, drops old bit,
        # picks new bit, confirms with sensor.
        self.replace_bit()
        return self.feeder.bit_present

    # --- Multi-step tightening protocol ------------------------------

    def tighten_screw(self, screw_index: int,
                      get_torque_fn, get_angle_fn,
                      drive_fn, stop_fn) -> ScrewResult:
        """Execute the full approach → snug → final → verify sequence.

        Parameters
        ----------
        screw_index : int
            0-based index of the screw in the current pattern.
        get_torque_fn : callable() -> float
            Returns live torque in Nm.
        get_angle_fn : callable() -> float
            Returns cumulative angle in degrees.
        drive_fn : callable(speed_rpm: float) -> None
            Runs the spindle at the given speed.
        stop_fn : callable() -> None
            Stops the spindle.
        """
        t_start = time.monotonic()
        self._phase = TightenPhase.APPROACH
        retry = 0
        max_retries = self.spec["max_retries"]

        while retry <= max_retries:
            ok, torque, angle = self._execute_tightening(
                get_torque_fn, get_angle_fn, drive_fn, stop_fn)

            if ok:
                break
            retry += 1
            logger.warning("Screw %d retry %d/%d",
                           screw_index, retry, max_retries)
            # Reverse slightly before retry
            self._phase = TightenPhase.APPROACH

        duration_ms = int((time.monotonic() - t_start) * 1000)
        result = ScrewResult(
            screw_index=screw_index,
            torque_nm=torque,
            angle_deg=angle,
            ok=ok,
            duration_ms=duration_ms,
            retry_count=retry,
        )

        self._record_result(result)
        self._phase = TightenPhase.DONE if ok else TightenPhase.ERROR
        return result

    def _execute_tightening(self, get_torque_fn, get_angle_fn,
                            drive_fn, stop_fn
                            ) -> tuple[bool, float, float]:
        """Run through the four tightening phases."""
        target_torque = self.spec["target_torque_nm"]
        torque_tol = self.spec["torque_tolerance_nm"]
        target_angle = self.spec["target_angle_deg"]
        angle_tol = self.spec["angle_tolerance_deg"]
        snug_torque = target_torque * 0.3

        cumulative_angle = 0.0
        torque = 0.0

        # Phase 1: Approach — fast free-running
        self._phase = TightenPhase.APPROACH
        drive_fn(self.spec["approach_speed_rpm"])
        while True:
            torque = get_torque_fn()
            cumulative_angle = get_angle_fn()
            if torque >= snug_torque * 0.1:
                break
            if cumulative_angle > target_angle * 1.5:
                stop_fn()
                return False, torque, cumulative_angle
            time.sleep(0.002)

        # Phase 2: Snug — medium speed until snug torque
        self._phase = TightenPhase.SNUG
        drive_fn(self.spec["snug_speed_rpm"])
        while True:
            torque = get_torque_fn()
            cumulative_angle = get_angle_fn()
            if torque >= snug_torque:
                break
            if cumulative_angle > target_angle * 1.5:
                stop_fn()
                return False, torque, cumulative_angle
            time.sleep(0.002)

        # Phase 3: Final — slow speed to target torque
        self._phase = TightenPhase.FINAL
        drive_fn(self.spec["final_speed_rpm"])
        while True:
            torque = get_torque_fn()
            cumulative_angle = get_angle_fn()
            if torque >= target_torque:
                break
            if cumulative_angle > target_angle + angle_tol:
                stop_fn()
                return False, torque, cumulative_angle
            time.sleep(0.002)
        stop_fn()

        # Phase 4: Verify — small reverse to confirm clamp
        self._phase = TightenPhase.VERIFY
        verify_angle = self.spec["verify_reverse_deg"]
        # In production: reverse spindle by verify_angle degrees, then
        # confirm torque breakaway is above threshold.

        # Evaluate pass/fail
        torque_ok = abs(torque - target_torque) <= torque_tol
        angle_ok = abs(cumulative_angle - target_angle) <= angle_tol
        ok = torque_ok and angle_ok

        return ok, torque, cumulative_angle

    # --- Result recording & statistics --------------------------------

    def _record_result(self, result: ScrewResult) -> None:
        self._results.append(result)
        self.spc.record(result.torque_nm, result.angle_deg)
        self.consume_screw()
        if result.ok:
            self._total_ok += 1
        else:
            self._total_nok += 1

    @property
    def last_results(self) -> list[ScrewResult]:
        return list(self._results)

    @property
    def ok_rate(self) -> float:
        total = self._total_ok + self._total_nok
        return self._total_ok / total if total > 0 else 0.0

    def get_spc_summary(self) -> dict:
        """Return a snapshot of SPC statistics."""
        return {
            "profile": self.profile.value,
            "total_ok": self._total_ok,
            "total_nok": self._total_nok,
            "ok_rate": self.ok_rate,
            "torque_cpk": self.spc.torque_cpk,
            "angle_cpk": self.spc.angle_cpk,
            "feeder_remaining": self.feeder.remaining,
            "bit_cycles": self.feeder.bit_cycles,
            "bit_change_needed": self.feeder.bit_change_needed,
        }

    @property
    def phase(self) -> TightenPhase:
        return self._phase
