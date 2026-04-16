"""
OP08#-1 Harmonic Reducer Motor Assembly Cell — Material Handler

Material flow management for tray stackers, conveyor belts, C-shape
rack, and part-tracking (barcode / QR → MES).  Ensures continuous
operation by managing buffer levels and automating tray swaps.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from src.data_types.cell_types import TraySlot, TrayType
from src.global_variables.system_config import (
    C_RACK_POSITIONS,
    TRAY_COLS,
    TRAY_ROWS,
    TRAY_SLOT_PITCH_MM,
    TRAY_STACKER_COUNT,
    TRAYS_PER_STACKER,
)

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Supporting types
# ---------------------------------------------------------------------------

class ConveyorDirection(Enum):
    UPSTREAM = "upstream"
    DOWNSTREAM = "downstream"


class StackerPosition(Enum):
    TOP = "top"
    MIDDLE = "middle"
    BOTTOM = "bottom"


@dataclass
class Tray:
    """Physical tray descriptor."""
    tray_id: str
    tray_type: TrayType
    barcode: str = ""
    slots: list[TraySlot] = field(default_factory=list)
    rows: int = TRAY_ROWS
    cols: int = TRAY_COLS

    def __post_init__(self) -> None:
        if not self.slots:
            self.slots = [
                TraySlot(row=r, col=c, occupied=True,
                         part_type=self.tray_type)
                for r in range(self.rows) for c in range(self.cols)
            ]

    @property
    def total_slots(self) -> int:
        return self.rows * self.cols

    @property
    def occupied_count(self) -> int:
        return sum(1 for s in self.slots if s.occupied)

    @property
    def is_empty(self) -> bool:
        return self.occupied_count == 0

    @property
    def is_full(self) -> bool:
        return self.occupied_count == self.total_slots

    def pick_next(self) -> Optional[TraySlot]:
        """Return the next occupied slot (row-major order) and mark it
        as empty.  Returns None when the tray is empty."""
        for slot in self.slots:
            if slot.occupied:
                slot.occupied = False
                return slot
        return None

    def place_next(self) -> Optional[TraySlot]:
        """Return the next empty slot and mark it as occupied."""
        for slot in self.slots:
            if not slot.occupied:
                slot.occupied = True
                return slot
        return None


@dataclass
class TrayStacker:
    """A single tray stacker column."""
    stacker_id: int
    tray_type: TrayType
    trays: list[Tray] = field(default_factory=list)
    capacity: int = TRAYS_PER_STACKER

    @property
    def tray_count(self) -> int:
        return len(self.trays)

    @property
    def is_full(self) -> bool:
        return self.tray_count >= self.capacity

    @property
    def is_empty(self) -> bool:
        return self.tray_count == 0

    @property
    def current_tray(self) -> Optional[Tray]:
        return self.trays[-1] if self.trays else None

    def push_tray(self, tray: Tray) -> bool:
        if self.is_full:
            logger.warning("Stacker %d full — cannot push tray",
                           self.stacker_id)
            return False
        self.trays.append(tray)
        return True

    def pop_tray(self) -> Optional[Tray]:
        if self.is_empty:
            return None
        return self.trays.pop()


@dataclass
class CRackSlot:
    """One slot on the C-shape storage rack."""
    position: int
    occupied: bool = False
    tray: Optional[Tray] = None


# ---------------------------------------------------------------------------
# Conveyor interface
# ---------------------------------------------------------------------------

class ConveyorBelt:
    """Simple conveyor belt start / stop / sensor interface."""

    def __init__(self, direction: ConveyorDirection) -> None:
        self.direction = direction
        self._running: bool = False
        self._part_present: bool = False

    def start(self) -> None:
        self._running = True
        logger.info("Conveyor %s started", self.direction.value)

    def stop(self) -> None:
        self._running = False
        logger.info("Conveyor %s stopped", self.direction.value)

    @property
    def is_running(self) -> bool:
        return self._running

    def read_sensor(self) -> bool:
        """Return True if a part / tray is detected at the transfer
        position."""
        return self._part_present

    def set_sensor_state(self, present: bool) -> None:
        """Simulation helper to inject sensor state."""
        self._part_present = present


# ---------------------------------------------------------------------------
# Material handler
# ---------------------------------------------------------------------------

class MaterialHandler:
    """Coordinates tray stackers, conveyors, C-rack, and MES
    part-tracking for continuous material supply."""

    def __init__(self) -> None:
        # Stackers
        self.stackers: dict[int, TrayStacker] = {}
        for i in range(TRAY_STACKER_COUNT):
            self.stackers[i] = TrayStacker(stacker_id=i,
                                           tray_type=TrayType.MOTOR_MOUNT)
        # C-rack
        self.c_rack: list[CRackSlot] = [
            CRackSlot(position=p) for p in range(C_RACK_POSITIONS)
        ]
        # Conveyors
        self.upstream_conveyor = ConveyorBelt(ConveyorDirection.UPSTREAM)
        self.downstream_conveyor = ConveyorBelt(ConveyorDirection.DOWNSTREAM)
        # Tracking
        self._part_log: list[dict] = []
        self._active_tray: Optional[Tray] = None

        logger.info("MaterialHandler initialised: %d stackers, "
                     "%d C-rack slots",
                     TRAY_STACKER_COUNT, C_RACK_POSITIONS)

    # --- Stacker operations -------------------------------------------

    def load_stacker(self, stacker_id: int,
                     tray_type: TrayType,
                     tray_count: int | None = None) -> bool:
        """Load fresh trays into a stacker (operator action)."""
        stacker = self.stackers.get(stacker_id)
        if stacker is None:
            logger.error("Unknown stacker ID %d", stacker_id)
            return False
        stacker.tray_type = tray_type
        count = tray_count or stacker.capacity
        for i in range(count):
            if stacker.is_full:
                break
            tray = Tray(
                tray_id=f"T{stacker_id}-{i:03d}",
                tray_type=tray_type,
                barcode=f"BC-{stacker_id}-{i:03d}",
            )
            stacker.push_tray(tray)
        logger.info("Stacker %d loaded with %d %s trays",
                     stacker_id, stacker.tray_count, tray_type.value)
        return True

    def get_next_part(self, stacker_id: int
                      ) -> Optional[tuple[Tray, TraySlot]]:
        """Get the next part from the stacker's current tray.

        Automatically swaps to the next tray when the current one is
        empty.  Returns (tray, slot) or None if the stacker is
        exhausted.
        """
        stacker = self.stackers.get(stacker_id)
        if stacker is None:
            return None

        tray = stacker.current_tray
        if tray is None:
            logger.warning("Stacker %d has no trays", stacker_id)
            return None

        slot = tray.pick_next()
        if slot is None:
            # Current tray is empty — swap
            logger.info("Tray %s empty — swapping", tray.tray_id)
            self._move_empty_tray_to_rack(stacker.pop_tray())
            tray = stacker.current_tray
            if tray is None:
                return None
            slot = tray.pick_next()
            if slot is None:
                return None

        self._log_part_pick(tray, slot)
        return tray, slot

    # --- Empty / full tray swap sequence ------------------------------

    def _move_empty_tray_to_rack(self, tray: Optional[Tray]) -> bool:
        """Place an empty tray onto the C-rack for operator removal."""
        if tray is None:
            return False
        for rack_slot in self.c_rack:
            if not rack_slot.occupied:
                rack_slot.occupied = True
                rack_slot.tray = tray
                logger.info("Empty tray %s placed on C-rack pos %d",
                            tray.tray_id, rack_slot.position)
                return True
        logger.warning("C-rack full — cannot store empty tray %s",
                       tray.tray_id)
        return False

    def clear_c_rack_slot(self, position: int) -> Optional[Tray]:
        """Operator removes a tray from the C-rack."""
        if position < 0 or position >= len(self.c_rack):
            return None
        slot = self.c_rack[position]
        if not slot.occupied:
            return None
        tray = slot.tray
        slot.occupied = False
        slot.tray = None
        logger.info("C-rack pos %d cleared (tray %s)",
                     position, tray.tray_id if tray else "?")
        return tray

    def empty_full_tray_swap(self, stacker_id: int) -> bool:
        """Full swap sequence: remove empty tray, load a new full tray
        from the upstream conveyor if available."""
        stacker = self.stackers.get(stacker_id)
        if stacker is None:
            return False

        # Remove current empty tray
        current = stacker.current_tray
        if current is not None and current.is_empty:
            self._move_empty_tray_to_rack(stacker.pop_tray())

        # Check upstream conveyor for incoming tray
        if not self.upstream_conveyor.read_sensor():
            logger.info("No tray on upstream conveyor for stacker %d",
                         stacker_id)
            return False

        # Simulate picking tray from conveyor
        new_tray = Tray(
            tray_id=f"T{stacker_id}-NEW-{time.monotonic_ns() % 10000:04d}",
            tray_type=stacker.tray_type,
            barcode=f"BC-NEW-{time.monotonic_ns() % 10000:04d}",
        )
        stacker.push_tray(new_tray)
        self.upstream_conveyor.set_sensor_state(False)
        logger.info("New tray %s loaded into stacker %d from conveyor",
                     new_tray.tray_id, stacker_id)
        return True

    # --- Conveyor control ---------------------------------------------

    def start_upstream(self) -> None:
        self.upstream_conveyor.start()

    def stop_upstream(self) -> None:
        self.upstream_conveyor.stop()

    def start_downstream(self) -> None:
        self.downstream_conveyor.start()

    def stop_downstream(self) -> None:
        self.downstream_conveyor.stop()

    # --- Part tracking / MES ------------------------------------------

    def _log_part_pick(self, tray: Tray, slot: TraySlot) -> None:
        record = {
            "tray_id": tray.tray_id,
            "barcode": tray.barcode,
            "part_type": tray.tray_type.value,
            "row": slot.row,
            "col": slot.col,
            "timestamp": time.time(),
        }
        self._part_log.append(record)

    def get_part_log(self) -> list[dict]:
        return list(self._part_log)

    def report_to_mes(self, mes_url: str) -> bool:
        """Push accumulated part-tracking records to MES via REST."""
        if not self._part_log:
            return True
        # Production: requests.post(mes_url, json=self._part_log)
        logger.info("Reported %d records to MES at %s",
                     len(self._part_log), mes_url)
        self._part_log.clear()
        return True

    # --- Buffer management --------------------------------------------

    def buffer_status(self) -> dict[int, dict]:
        """Return a summary of all stacker buffer levels."""
        status: dict[int, dict] = {}
        for sid, stacker in self.stackers.items():
            current = stacker.current_tray
            status[sid] = {
                "tray_count": stacker.tray_count,
                "capacity": stacker.capacity,
                "current_tray_occupancy": (
                    current.occupied_count if current else 0),
                "tray_type": stacker.tray_type.value,
                "needs_refill": stacker.tray_count <= 1,
            }
        return status

    def c_rack_status(self) -> list[dict]:
        return [
            {
                "position": s.position,
                "occupied": s.occupied,
                "tray_id": s.tray.tray_id if s.tray else None,
            }
            for s in self.c_rack
        ]
