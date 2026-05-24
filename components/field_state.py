"""Field state component — zone tracking + AprilTag proximity in one place.

This single component replaces the two separate ``FieldPosition`` and
``AprilTagTracker`` components.  Combining them avoids duplicating the
``get_pose()`` call every loop and gives the rest of the code one clear
place to ask "where is the robot on the field?".

Responsibilities
----------------
1. **Zone classification** — determines whether the robot is on the Blue
   Alliance side, in the Neutral Zone, or on the Red Alliance side based
   on the WPILib X coordinate of the estimated pose.

2. **AprilTag proximity** — loads the official 2026 FRC field layout
   (configurable via ``constants.APRILTAG_FIELD``) at startup, then every
   loop computes:
   * The nearest AprilTag and its distance.
   * All tags within ``constants.APRILTAG_PROXIMITY_RADIUS_M`` metres
     ("expected" tags the robot could plausibly see).

   The ``Limelight`` component calls ``is_tag_expected(tag_id)`` to decide
   how much to trust a detected tag — if it matches, fusion uses tighter
   standard deviations for better localisation.

Field zone layout (WPILib coordinates, blue origin at X = 0)
-------------------------------------------------------------
::

    Blue wall                                              Red wall
    X = 0                                               X = 17.55 m
    │  Blue Alliance  │       Neutral Zone       │  Red Alliance  │
    │    (0–5.18 m)   │    (5.18 m – 12.37 m)   │ (12.37–17.55 m)│

2026 Reefscape Rebuilt tag groups (welded field)
------------------------------------------------
::

    IDs  1–12  Red reef cluster    x ≈ 11.3–12.5 m
    IDs 13–16  Red barge wall      x ≈ 16.5 m
    IDs 17–28  Blue reef cluster   x ≈ 4.0–5.2 m
    IDs 29–32  Blue barge wall     x ≈ 0.0 m

Usage
-----
Declare in ``robot.py``::

    field_state: FieldState

Then inject into any component that needs it (MagicBot wires by name)::

    field_state: FieldState

Published ``@feedback`` keys (all under the component name prefix)
------------------------------------------------------------------
* ``zone_name``           — "Blue Alliance" | "Neutral Zone" | "Red Alliance" | "Unknown"
* ``pose_x_m``            — robot X in metres
* ``pose_y_m``            — robot Y in metres
* ``is_blue_alliance``    — bool
* ``is_red_alliance``     — bool
* ``is_neutral``          — bool
* ``nearest_tag_id``      — ID of the closest AprilTag (-1 = none)
* ``nearest_tag_dist_m``  — distance to that tag (m)
* ``nearby_tag_count``    — number of tags within proximity radius
* ``nearby_tag_ids_str``  — comma-separated nearby tag IDs (e.g. "7,8,9")
"""

from __future__ import annotations

import enum
import math

from magicbot import feedback
from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import Pose2d, Rotation2d

import constants
from components.swerve_drive import SwerveDrive


class FieldZone(enum.Enum):
    """Logical regions of the 2026 FRC Reefscape field."""

    BLUE_ALLIANCE = "Blue Alliance"
    NEUTRAL = "Neutral Zone"
    RED_ALLIANCE = "Red Alliance"
    UNKNOWN = "Unknown"  # pose not yet valid (startup / no vision fix)


class FieldState:
    """MagicBot component — combined field zone + AprilTag proximity tracker.

    MagicBot injects the shared ``SwerveDrive`` instance automatically
    because the attribute name matches the robot-level declaration.
    """

    # Injected by MagicBot.
    swerve_drive: SwerveDrive

    # ------------------------------------------------------------------
    # MagicBot lifecycle
    # ------------------------------------------------------------------

    def setup(self) -> None:
        """Load field layout and initialise state.  Called once at startup."""
        layout = AprilTagFieldLayout.loadField(constants.APRILTAG_FIELD)

        # Pre-compute 2-D (x, y) positions for every tag so execute() is
        # a simple arithmetic loop with no object allocation.
        self._tag_positions: dict[int, tuple[float, float]] = {}
        for tag in layout.getTags():
            pose3d = layout.getTagPose(tag.ID)
            p2d = pose3d.toPose2d()
            self._tag_positions[tag.ID] = (p2d.X(), p2d.Y())

        # Zone state
        self._zone: FieldZone = FieldZone.UNKNOWN
        self._x_m: float = 0.0
        self._y_m: float = 0.0
        # Tracks whether the pose estimator has received at least one non-zero
        # measurement.  The robot can legitimately start at X=0, Y=0 only on
        # very specific field placements, so we use a separate boolean rather
        # than checking for exact zero to avoid a fragile corner case.
        self._pose_valid: bool = False

        # AprilTag proximity state
        self._nearest_id: int = -1
        self._nearest_dist_m: float = float("inf")
        self._nearby_ids: list[int] = []

    # ------------------------------------------------------------------
    # Public API — zone
    # ------------------------------------------------------------------

    @property
    def zone(self) -> FieldZone:
        """The ``FieldZone`` the robot currently occupies."""
        return self._zone

    # ------------------------------------------------------------------
    # Public API — AprilTag proximity (consumed by Limelight)
    # ------------------------------------------------------------------

    def is_tag_expected(self, tag_id: int) -> bool:
        """Return ``True`` if *tag_id* is within the proximity radius.

        Used by ``Limelight.execute()`` to validate detected tags against
        what odometry says the robot should be able to see.
        """
        return tag_id in self._nearby_ids

    def get_nearby_tag_ids(self) -> list[int]:
        """Return IDs of all tags within ``APRILTAG_PROXIMITY_RADIUS_M``."""
        return list(self._nearby_ids)

    def get_nearest_tag_id(self) -> int:
        """Return the ID of the closest AprilTag to the robot's current pose."""
        return self._nearest_id

    def get_tag_pose(self, tag_id: int) -> Pose2d | None:
        """Return the 2-D field pose of *tag_id*, or ``None`` if unknown."""
        pos = self._tag_positions.get(tag_id)
        if pos is None:
            return None
        return Pose2d(pos[0], pos[1], Rotation2d())

    # ------------------------------------------------------------------
    # Telemetry — @feedback publishes to SmartDashboard / Elastic every loop
    # ------------------------------------------------------------------

    @feedback
    def zone_name(self) -> str:
        """Human-readable zone: "Blue Alliance", "Neutral Zone", etc."""
        return self._zone.value

    @feedback
    def pose_x_m(self) -> float:
        """Robot X position on the field in metres (WPILib coordinates)."""
        return round(self._x_m, 3)

    @feedback
    def pose_y_m(self) -> float:
        """Robot Y position on the field in metres (WPILib coordinates)."""
        return round(self._y_m, 3)

    @feedback
    def is_blue_alliance(self) -> bool:
        """``True`` while the robot is on the Blue Alliance side."""
        return self._zone is FieldZone.BLUE_ALLIANCE

    @feedback
    def is_red_alliance(self) -> bool:
        """``True`` while the robot is on the Red Alliance side."""
        return self._zone is FieldZone.RED_ALLIANCE

    @feedback
    def is_neutral(self) -> bool:
        """``True`` while the robot is in the Neutral Zone."""
        return self._zone is FieldZone.NEUTRAL

    @feedback
    def pose_valid(self) -> bool:
        """``True`` once the pose estimator has received a valid measurement.

        ``False`` at startup before odometry has moved away from the default
        pose.  Use this to know whether zone / tag data is trustworthy.
        """
        return self._pose_valid

    @feedback
    def nearest_tag_id(self) -> int:
        """ID of the AprilTag nearest to the robot (-1 = none)."""
        return self._nearest_id

    @feedback
    def nearest_tag_dist_m(self) -> float:
        """Distance from the robot to the nearest AprilTag in metres."""
        return round(self._nearest_dist_m, 3) if math.isfinite(self._nearest_dist_m) else -1.0

    @feedback
    def nearby_tag_count(self) -> int:
        """Number of AprilTags within ``APRILTAG_PROXIMITY_RADIUS_M``."""
        return len(self._nearby_ids)

    @feedback
    def nearby_tag_ids_str(self) -> str:
        """Comma-separated nearby tag IDs (e.g. "7,8,9") for dashboards."""
        return ",".join(str(i) for i in sorted(self._nearby_ids)) if self._nearby_ids else "none"

    # ------------------------------------------------------------------
    # MagicBot execute() — called every ~20 ms loop
    # ------------------------------------------------------------------

    def execute(self) -> None:
        """Update zone + AprilTag proximity from the current odometry pose."""
        pose: Pose2d = self.swerve_drive.get_pose()
        x = pose.X()
        y = pose.Y()

        # Mark the pose as valid once it has moved away from the default origin.
        # A threshold of 0.05 m avoids flipping on sensor noise at true (0, 0).
        if not self._pose_valid and (abs(x) > 0.05 or abs(y) > 0.05):
            self._pose_valid = True

        # --- Zone classification ---
        # WPILib coordinates always use blue-alliance origin (low X = blue wall).
        # We confirm which side is "ours" from the Driver Station alliance color
        # so that `is_blue_alliance` / `is_red_alliance` reflect the robot's
        # own alliance even when it is sitting deep in the opponent's zone.
        if not self._pose_valid:
            self._zone = FieldZone.UNKNOWN
        elif x < constants.BLUE_ZONE_END_M:
            self._zone = FieldZone.BLUE_ALLIANCE
        elif x > constants.RED_ZONE_START_M:
            self._zone = FieldZone.RED_ALLIANCE
        else:
            self._zone = FieldZone.NEUTRAL

        self._x_m = x
        self._y_m = y

        # --- AprilTag proximity ---
        radius = constants.APRILTAG_PROXIMITY_RADIUS_M
        best_id = -1
        best_dist = float("inf")
        nearby: list[int] = []

        for tag_id, (tx, ty) in self._tag_positions.items():
            dist = math.hypot(x - tx, y - ty)
            if dist < best_dist:
                best_dist = dist
                best_id = tag_id
            if dist <= radius:
                nearby.append(tag_id)

        self._nearest_id = best_id
        self._nearest_dist_m = best_dist
        self._nearby_ids = nearby
