"""
OP08#-1 Harmonic Reducer Motor Assembly Cell — Vision System

2-D machine-vision module for part localisation, screw-hole detection,
and tray-slot occupancy.  Supports GigE Vision and USB3 cameras with
hand-eye calibration for both eye-in-hand (R1) and eye-to-hand
(worktable overhead) setups.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np

from src.data_types.cell_types import (
    CalibrationMode,
    Pose,
    TraySlot,
    TrayType,
    VisionOffset,
)

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Supporting types
# ---------------------------------------------------------------------------

class PartTemplate(Enum):
    """Template IDs for template-matching."""
    MOTOR_MOUNT = "motor_mount"
    MOTOR = "motor"
    BRAKE_MAGNET_BASE = "brake_magnet_base"
    SCREW_HOLE = "screw_hole"


@dataclass
class DetectionResult:
    """Single object detection from image processing."""
    template: PartTemplate
    centre_px: tuple[float, float]
    angle_deg: float
    score: float
    bounding_box: tuple[int, int, int, int] = (0, 0, 0, 0)


@dataclass
class CalibrationData:
    """Hand-eye calibration parameters."""
    mode: CalibrationMode
    camera_matrix: np.ndarray = field(
        default_factory=lambda: np.eye(3, dtype=np.float64))
    dist_coeffs: np.ndarray = field(
        default_factory=lambda: np.zeros(5, dtype=np.float64))
    hand_eye_matrix: np.ndarray = field(
        default_factory=lambda: np.eye(4, dtype=np.float64))
    px_per_mm: float = 10.0
    reprojection_error_px: float = 0.0


# ---------------------------------------------------------------------------
# Vision system
# ---------------------------------------------------------------------------

class VisionSystem:
    """Camera interface, part detection, and pose estimation pipeline."""

    def __init__(self, camera_id: str, config: dict) -> None:
        self.camera_id = camera_id
        self.config = config
        self.ip_address: str = config.get("ip_address", "")
        self.resolution: tuple[int, int] = tuple(config.get("resolution",
                                                             (1920, 1200)))
        self.exposure_us: int = config.get("exposure_us", 5000)
        self.gain_db: float = config.get("gain_db", 6.0)
        self._calibration: Optional[CalibrationData] = None
        self._templates: dict[PartTemplate, np.ndarray] = {}
        self._connected: bool = False
        self._last_image: Optional[np.ndarray] = None
        logger.info("VisionSystem '%s' initialised (ip=%s, res=%s)",
                     camera_id, self.ip_address, self.resolution)

    # --- Connection ---------------------------------------------------

    def connect(self) -> bool:
        """Open the GigE / USB3 camera stream."""
        try:
            # Production code would use Harvester / pypylon / etc.
            self._connected = True
            logger.info("Camera '%s' connected at %s",
                         self.camera_id, self.ip_address)
            return True
        except Exception:
            logger.exception("Camera '%s' connection failed", self.camera_id)
            return False

    def disconnect(self) -> None:
        self._connected = False
        logger.info("Camera '%s' disconnected", self.camera_id)

    @property
    def is_connected(self) -> bool:
        return self._connected

    # --- Image acquisition --------------------------------------------

    def grab_image(self) -> Optional[np.ndarray]:
        """Acquire a single frame from the camera."""
        if not self._connected:
            logger.error("Camera '%s' not connected", self.camera_id)
            return None
        # Placeholder: return a synthetic image for offline testing
        h, w = self.resolution[1], self.resolution[0]
        self._last_image = np.zeros((h, w), dtype=np.uint8)
        return self._last_image

    def set_exposure(self, exposure_us: int) -> None:
        self.exposure_us = exposure_us
        logger.debug("Camera '%s' exposure set to %d us",
                      self.camera_id, exposure_us)

    def set_gain(self, gain_db: float) -> None:
        self.gain_db = gain_db

    # --- Template management ------------------------------------------

    def load_template(self, template_id: PartTemplate,
                      image: np.ndarray) -> None:
        """Register a reference template image for matching."""
        self._templates[template_id] = image
        logger.info("Template '%s' loaded (%s)",
                     template_id.value, image.shape)

    # --- Part detection -----------------------------------------------

    def detect_part(self, template_id: PartTemplate,
                    min_score: float = 0.80
                    ) -> Optional[DetectionResult]:
        """Run normalised cross-correlation template matching."""
        image = self._last_image
        if image is None:
            image = self.grab_image()
        if image is None:
            return None

        template = self._templates.get(template_id)
        if template is None:
            logger.error("No template loaded for '%s'", template_id.value)
            return None

        # --- Core matching (cv2.matchTemplate in production) ----------
        # Placeholder: simulate a detection at the image centre
        cx = image.shape[1] / 2.0
        cy = image.shape[0] / 2.0
        score = 0.95
        angle = 0.0

        if score < min_score:
            logger.warning("Score %.3f below threshold %.3f for '%s'",
                           score, min_score, template_id.value)
            return None

        result = DetectionResult(
            template=template_id,
            centre_px=(cx, cy),
            angle_deg=angle,
            score=score,
        )
        logger.debug("Detected '%s' at (%.1f, %.1f) score=%.3f",
                      template_id.value, cx, cy, score)
        return result

    # --- Screw-hole detection -----------------------------------------

    def detect_screw_holes(self, expected_count: int = 4,
                           min_radius_px: int = 8,
                           max_radius_px: int = 20
                           ) -> list[tuple[float, float, float]]:
        """Detect circular screw holes using Hough circle transform.

        Returns a list of (cx, cy, radius) in pixel coordinates.
        """
        image = self._last_image
        if image is None:
            image = self.grab_image()
        if image is None:
            return []

        # Production: cv2.HoughCircles(image, cv2.HOUGH_GRADIENT, ...)
        # Placeholder: return synthetic hole positions arranged in a square
        w, h = image.shape[1], image.shape[0]
        spacing = 60.0
        base_x, base_y = w / 2.0 - spacing / 2, h / 2.0 - spacing / 2
        holes: list[tuple[float, float, float]] = []
        for row in range(2):
            for col in range(2):
                if len(holes) >= expected_count:
                    break
                holes.append((base_x + col * spacing,
                              base_y + row * spacing,
                              12.0))

        logger.info("Detected %d screw holes (expected %d)",
                     len(holes), expected_count)
        return holes

    # --- Pose estimation (pixel → world) ------------------------------

    def pixel_to_world(self, px_x: float, px_y: float,
                       angle_deg: float = 0.0) -> VisionOffset:
        """Convert pixel-space detection to a world-frame offset.

        The offset is relative to the calibrated reference point so the
        robot can apply it as a correction to the nominal pose.
        """
        if self._calibration is None:
            raise RuntimeError(f"Camera '{self.camera_id}' not calibrated")

        scale = 1.0 / self._calibration.px_per_mm   # mm per pixel
        ref_x = self.resolution[0] / 2.0
        ref_y = self.resolution[1] / 2.0

        dx_mm = (px_x - ref_x) * scale
        dy_mm = (px_y - ref_y) * scale

        # Apply hand-eye rotation to map camera frame → robot base frame
        R = self._calibration.hand_eye_matrix[:2, :2]
        world_offset = R @ np.array([dx_mm, dy_mm])

        return VisionOffset(
            dx=float(world_offset[0]),
            dy=float(world_offset[1]),
            drz=angle_deg,
            confidence=0.95,
        )

    def estimate_part_pose(self, template_id: PartTemplate
                           ) -> Optional[VisionOffset]:
        """Full pipeline: grab → detect → convert to world offset."""
        self.grab_image()
        det = self.detect_part(template_id)
        if det is None:
            return None
        return self.pixel_to_world(det.centre_px[0],
                                   det.centre_px[1],
                                   det.angle_deg)

    # --- Tray slot occupancy ------------------------------------------

    def detect_tray_occupancy(self, rows: int, cols: int,
                              slot_pitch_mm: float,
                              part_type: TrayType
                              ) -> list[TraySlot]:
        """Check which tray slots contain a part."""
        self.grab_image()
        slots: list[TraySlot] = []
        for r in range(rows):
            for c in range(cols):
                # Production: ROI extraction + threshold on mean intensity
                occupied = True  # placeholder
                slots.append(TraySlot(
                    row=r, col=c, occupied=occupied, part_type=part_type))
        occupied_count = sum(1 for s in slots if s.occupied)
        logger.info("Tray occupancy: %d / %d slots filled",
                     occupied_count, len(slots))
        return slots

    # --- Calibration --------------------------------------------------

    def load_calibration(self, calib: CalibrationData) -> None:
        """Load pre-computed calibration parameters."""
        self._calibration = calib
        logger.info("Calibration loaded for '%s' (mode=%s, err=%.3f px)",
                     self.camera_id, calib.mode.value,
                     calib.reprojection_error_px)

    def run_hand_eye_calibration(
            self,
            robot_poses: list[Pose],
            board_images: list[np.ndarray],
            mode: CalibrationMode,
            board_size: tuple[int, int] = (7, 5),
            square_size_mm: float = 10.0,
    ) -> CalibrationData:
        """Compute hand-eye calibration from a set of robot poses and
        corresponding checkerboard images.

        In production this wraps ``cv2.calibrateHandEye``.
        """
        if len(robot_poses) != len(board_images):
            raise ValueError("Pose count must equal image count")
        if len(robot_poses) < 10:
            logger.warning("Fewer than 10 calibration poses — accuracy "
                           "may be low")

        # Placeholder: identity transform
        calib = CalibrationData(
            mode=mode,
            px_per_mm=self.config.get("focal_length_mm", 12.0)
                      / (self.config.get("pixel_size_um", 3.45) / 1000.0),
            reprojection_error_px=0.12,
        )
        self._calibration = calib
        logger.info("Hand-eye calibration complete (mode=%s, rms=%.3f px)",
                     mode.value, calib.reprojection_error_px)
        return calib
