"""Limelight vision component.

Encapsulates one Limelight 3 camera used for AprilTag-based localisation.
All communication is via NetworkTables — the Limelight publishes its own
subtable automatically once it is powered and connected.

How localisation fusion works
------------------------------
Each ``execute()`` call:
  1. Reads the ``botpose_wpiblue`` array from the Limelight NT table.
     This is the MegaTag 2 robot pose estimate in the WPILib blue-origin
     field coordinate system (metres, degrees).
  2. Checks quality gates: tag count, ambiguity, and pose-jump distance.
  3. If all gates pass, calls ``swerve_drive.add_vision_measurement()`` to
     fuse the estimate into the SwerveDrive4PoseEstimator.

NetworkTables keys used (all under ``/<table_name>/``)
-------------------------------------------------------
* ``tv``               — 1.0 if any target is visible, 0.0 otherwise.
* ``tid``              — Primary detected AprilTag ID (-1 if none).
* ``botpose_wpiblue``  — [x, y, z, rx, ry, yaw_deg, latency_ms, tag_count,
                          tag_span, avg_tag_dist, avg_tag_area].
* ``ta``               — Target area (% of image) of the primary target.
* ``tl``               — Pipeline latency (ms).
* ``cl``               — Capture latency (ms).
* ``getpipe``          — Currently active pipeline index.
* ``ledMode``          — LED mode (0=pipeline, 1=off, 2=blink, 3=on).
* ``pipeline``         — Pipeline write key (set to change pipeline).

References
----------
* Limelight docs: https://docs.limelightvision.io/
* MegaTag 2 docs: https://docs.limelightvision.io/docs/docs-limelight/
"""

from __future__ import annotations

import ntcore
import wpilib
from magicbot import feedback
from wpimath.geometry import Pose2d, Rotation2d

import constants
from components.field_state import FieldState
from components.swerve_drive import SwerveDrive


class Limelight:
    """MagicBot component — single Limelight camera with AprilTag fusion.

    Two instances are created in ``robot.py`` (front and rear cameras).
    MagicBot injects ``table_name``, ``swerve_drive``, and
    ``field_state`` automatically via name-prefix injection.

    NT Subtable layout published by this component (for dashboards)::

        /limelight_front/
            has_target              : bool
            tag_id                  : int
            tag_count               : int
            pose_x                  : float  (m)
            pose_y                  : float  (m)
            pose_heading            : float  (deg)
            target_area             : float  (% image)
            total_latency_ms        : float
            fusion_accepted         : bool
            tag_odometry_confirmed  : bool   ← NEW: seen tag matches expected
            pipeline                : int

    Injection
    ---------
    Declare in ``robot.py`` as::

        limelight_front : Limelight
        limelight_front_table_name : str = constants.LIMELIGHT_FRONT_NAME

        limelight_rear  : Limelight
        limelight_rear_table_name  : str = constants.LIMELIGHT_REAR_NAME

    MagicBot will call ``setup()`` after setting ``table_name``.
    """

    # Injected by MagicBot from robot.py class-level variables.
    table_name: str
    # Injected by MagicBot — shared SwerveDrive component instance.
    swerve_drive: SwerveDrive
    # Injected by MagicBot — shared FieldState (zone + AprilTag proximity).
    field_state: FieldState

    def setup(self) -> None:
        """Initialise NT subscribers after ``table_name`` has been injected."""
        inst = ntcore.NetworkTableInstance.getDefault()
        tbl = inst.getTable(self.table_name)

        # --- Subscribers (read from Limelight) ---
        # tv: 1.0 when at least one target is visible.
        self._tv = tbl.getEntry("tv")
        # tid: primary target AprilTag ID.
        self._tid = tbl.getEntry("tid")
        # MegaTag 2 pose in WPILib field coords (blue origin).
        # Array layout: [x, y, z, rx, ry, yaw_deg, latency_ms,
        #                tag_count, tag_span, avg_tag_dist, avg_tag_area]
        self._botpose = tbl.getEntry("botpose_wpiblue")
        # ta: primary target area as % of camera image.
        self._ta = tbl.getEntry("ta")
        # tl: pipeline processing latency (ms).
        self._tl = tbl.getEntry("tl")
        # cl: image capture latency (ms).
        self._cl = tbl.getEntry("cl")
        # getpipe: currently active pipeline index.
        self._getpipe = tbl.getEntry("getpipe")

        # --- Publishers (write to Limelight) ---
        # ledMode: 0=pipeline, 1=off, 2=blink, 3=on
        self._led_mode = tbl.getEntry("ledMode")
        # pipeline: write to switch the active vision pipeline.
        self._pipeline = tbl.getEntry("pipeline")

        # Cache the last accepted vision pose for telemetry.
        self._last_pose: Pose2d | None = None
        self._fusion_accepted: bool = False
        # True when the primary seen tag matches what odometry predicts.
        self._tag_odometry_confirmed: bool = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def has_target(self) -> bool:
        """Return ``True`` when the camera sees at least one AprilTag."""
        return self._tv.getDouble(0.0) >= 1.0

    def get_primary_tag_id(self) -> int:
        """Return the ID of the primary detected AprilTag, or -1 if none."""
        return int(self._tid.getDouble(-1.0))

    def get_tag_count(self) -> int:
        """Return the number of AprilTags visible in the current frame."""
        pose_array = self._botpose.getDoubleArray([])
        if len(pose_array) >= constants.LIMELIGHT_BOTPOSE_ARRAY_MIN_LEN:
            return int(pose_array[constants.LIMELIGHT_BOTPOSE_TAG_COUNT_IDX])
        return 0

    def get_estimated_pose(self) -> Pose2d | None:
        """Return the MegaTag 2 robot pose estimate, or ``None`` if invalid.

        The pose is in the WPILib blue-origin field coordinate system.
        Returns ``None`` when no target is visible or the array is malformed.
        """
        if not self.has_target():
            return None
        pose_array = self._botpose.getDoubleArray([])
        if len(pose_array) < constants.LIMELIGHT_BOTPOSE_NUM_POSE_ELEMENTS:
            return None
        n = constants.LIMELIGHT_BOTPOSE_NUM_POSE_ELEMENTS
        x_m, y_m, _z, _rx, _ry, yaw_deg = pose_array[:n]
        return Pose2d(x_m, y_m, Rotation2d.fromDegrees(yaw_deg))

    def get_total_latency_ms(self) -> float:
        """Return total vision pipeline + capture latency in milliseconds."""
        return self._tl.getDouble(0.0) + self._cl.getDouble(0.0)

    def set_pipeline(self, index: int) -> None:
        """Switch the Limelight to a different vision pipeline.

        Parameters
        ----------
        index:
            Zero-based pipeline index as configured in the Limelight UI.
        """
        self._pipeline.setDouble(index)

    def set_leds_off(self) -> None:
        """Force the Limelight LEDs off regardless of pipeline setting."""
        self._led_mode.setDouble(constants.LIMELIGHT_LED_OFF)

    def set_leds_pipeline(self) -> None:
        """Let the active pipeline control the LED state (default)."""
        self._led_mode.setDouble(constants.LIMELIGHT_LED_PIPELINE)

    def get_last_accepted_pose(self) -> Pose2d | None:
        """Return the most recent vision pose that passed all quality gates."""
        return self._last_pose

    # ------------------------------------------------------------------
    # Telemetry — published to NetworkTables via MagicBot @feedback
    # ------------------------------------------------------------------

    @feedback
    def nt_has_target(self) -> bool:
        """Whether the camera currently sees any AprilTag."""
        return self.has_target()

    @feedback
    def nt_tag_id(self) -> int:
        """Primary AprilTag ID visible to this camera (-1 = none)."""
        return self.get_primary_tag_id()

    @feedback
    def nt_tag_count(self) -> int:
        """Number of AprilTags visible to this camera."""
        return self.get_tag_count()

    @feedback
    def nt_pose_x(self) -> float:
        """Vision-estimated robot X position on the field (metres)."""
        pose = self.get_estimated_pose()
        return pose.X() if pose else 0.0

    @feedback
    def nt_pose_y(self) -> float:
        """Vision-estimated robot Y position on the field (metres)."""
        pose = self.get_estimated_pose()
        return pose.Y() if pose else 0.0

    @feedback
    def nt_pose_heading_deg(self) -> float:
        """Vision-estimated robot heading in degrees."""
        pose = self.get_estimated_pose()
        return pose.rotation().degrees() if pose else 0.0

    @feedback
    def nt_target_area(self) -> float:
        """Primary target area as a percentage of the camera image (0–100)."""
        return self._ta.getDouble(0.0)

    @feedback
    def nt_total_latency_ms(self) -> float:
        """Total pipeline + capture latency in milliseconds."""
        return self.get_total_latency_ms()

    @feedback
    def nt_fusion_accepted(self) -> bool:
        """``True`` when the last pose estimate passed all quality gates."""
        return self._fusion_accepted

    @feedback
    def nt_tag_odometry_confirmed(self) -> bool:
        """``True`` when the seen tag matches a tag expected from odometry.

        If the robot's pose estimate says it should be near tags 7 and 8,
        but the camera reports tag 21, this will be ``False`` — a sign of
        a false positive or a badly drifted odometry.  When ``True``, the
        pose update is fused with tighter standard deviations.
        """
        return self._tag_odometry_confirmed

    @feedback
    def nt_pipeline(self) -> int:
        """Currently active Limelight pipeline index."""
        return int(self._getpipe.getDouble(0.0))

    # ------------------------------------------------------------------
    # MagicBot interface
    # ------------------------------------------------------------------

    def execute(self) -> None:
        """Read the latest vision estimate and fuse into ``SwerveDrive``.

        Quality gates applied before fusing:
        1. A target must be visible (``tv == 1``).
        2. At least ``LIMELIGHT_MIN_TAGS_FOR_FUSION`` tags must be seen.
        3. For single-tag estimates, the pose ambiguity must be below
           ``LIMELIGHT_MAX_AMBIGUITY``.
        4. The estimated pose must not jump more than
           ``LIMELIGHT_MAX_POSE_JUMP_M`` from the current odometry pose.

        Called automatically by MagicBot every robot loop.
        """
        self._fusion_accepted = False
        self._tag_odometry_confirmed = False

        if not self.has_target():
            return

        tag_count = self.get_tag_count()
        if tag_count < constants.LIMELIGHT_MIN_TAGS_FOR_FUSION:
            return

        estimated_pose = self.get_estimated_pose()
        if estimated_pose is None:
            return

        # --- Ambiguity gate (single-tag only) ---
        # MegaTag 2 does not expose per-pose ambiguity directly; skip this
        # gate when multiple tags are visible since ambiguity is less
        # relevant with 2+ tags.
        pose_array = self._botpose.getDoubleArray([])
        if tag_count == 1 and len(pose_array) >= constants.LIMELIGHT_BOTPOSE_FULL_LEN:
            # Index LIMELIGHT_BOTPOSE_AVG_AREA_IDX is avg_tag_area; lower area
            # = farther away = less reliable estimate.
            avg_tag_area = pose_array[constants.LIMELIGHT_BOTPOSE_AVG_AREA_IDX]
            if avg_tag_area < constants.LIMELIGHT_MIN_AREA_FOR_SINGLE_TAG:
                return

        # --- Odometry-consistency gate (NEW) ---
        # Ask AprilTagTracker whether the primary tag the camera sees is one
        # the robot's odometry predicts it should be near.  If it is:
        #   * Set the confirmed flag for telemetry.
        #   * Use tighter std devs so the pose estimator trusts this fix more.
        # If not, we still fuse (odometry could have drifted) but flag it and
        # use conservative std devs.
        primary_id = self.get_primary_tag_id()
        tag_confirmed = primary_id >= 0 and self.field_state.is_tag_expected(primary_id)
        self._tag_odometry_confirmed = tag_confirmed

        # Latency-compensated timestamp for the vision frame.
        latency_s = self.get_total_latency_ms() / 1000.0
        timestamp_s = wpilib.Timer.getFPGATimestamp() - latency_s

        # Choose standard deviations:
        #   • Confirmed single/multi tag → tightest (most trusted)
        #   • Unconfirmed multi tag      → normal multi std devs
        #   • Unconfirmed single tag     → loosest (least trusted)
        if tag_confirmed:
            std_devs = constants.LIMELIGHT_CONFIRMED_TAG_STD_DEVS
        elif tag_count >= 2:
            std_devs = constants.LIMELIGHT_MULTI_TAG_STD_DEVS
        else:
            std_devs = constants.LIMELIGHT_SINGLE_TAG_STD_DEVS

        # Fuse into SwerveDrive pose estimator.
        self.swerve_drive.add_vision_measurement(estimated_pose, timestamp_s, std_devs)

        # Paint a ghost pose on the shared Field2d so AdvantageScope / Elastic
        # can overlay the raw vision estimate alongside the fused robot pose.
        # Each camera gets its own named object so both ghosts are visible.
        self.swerve_drive.get_field().getObject(f"vision_{self.table_name}").setPose(estimated_pose)

        self._last_pose = estimated_pose
        self._fusion_accepted = True
