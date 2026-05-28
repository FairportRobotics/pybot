"""Robot-wide constants.

All physical measurements, CAN IDs, PID gains, and tuning parameters live
here so they can be found, audited, and changed in one place.

Unit conventions
----------------
* Distances : metres  (SI)
* Angles    : radians (SI)
* Speeds    : metres/second or radians/second
* Mass      : kilograms

For students
------------
If you see a number in the code that you don't understand, check this file
first!  Every "magic number" should have a name and a comment here so you
know exactly what it means and why it has that value.
"""

from __future__ import annotations

import math

# ===========================================================================
# Logging
# ===========================================================================

# Set to True to enable WPILib DataLogManager at robot startup.
# Logs all NetworkTables data + custom log entries to a timestamped
# .wpilog file.  On the real robot this goes to /home/lvuser/logs/.
# In simulation it writes to the current working directory.
# Set to False to save disk space during development (e.g., unit-test runs).
LOGGING_ENABLED: bool = False

# ===========================================================================
# Robot Timing
# ===========================================================================

# The robot's main loop runs at 50 Hz — it wakes up every 20 milliseconds
# (0.02 seconds) to read sensors, make decisions, and command motors.
ROBOT_LOOP_PERIOD_S: float = 0.02  # 20 ms = 0.02 s

# ===========================================================================
# Electrical
# ===========================================================================

# A fully-charged FRC robot battery supplies about 12.6 V, but we use 12 V
# as our nominal "full voltage" for safety calculations.
BATTERY_VOLTAGE_V: float = 12.0

# ===========================================================================
# CAN Bus
# ===========================================================================

# Name of the CANivore bus.  All Phoenix 6 devices are on this bus.
CANIVORE_BUS: str = "canivore"

# ---------------------------------------------------------------------------
# Swerve module CAN IDs
# Order: Front-Left, Front-Right, Back-Left, Back-Right
# Each module has a drive motor, a steer motor, and a CANcoder.
# ---------------------------------------------------------------------------
DRIVE_MOTOR_IDS: tuple[int, int, int, int] = (1, 2, 3, 4)
STEER_MOTOR_IDS: tuple[int, int, int, int] = (5, 6, 7, 8)
CANCODER_IDS: tuple[int, int, int, int] = (9, 10, 11, 12)

# Steer encoder absolute offsets (radians).
# Set these after physically aligning each module to its zero position.
CANCODER_OFFSETS_RAD: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)

# ---------------------------------------------------------------------------
# Other mechanism CAN IDs
# ---------------------------------------------------------------------------
INTAKE_MOTOR_ID: int = 13
SHOOTER_MOTOR_ID: int = 14
PIGEON_ID: int = 15

# ---------------------------------------------------------------------------
# Digital I/O ports (roboRIO DIO header)
# ---------------------------------------------------------------------------
# Beam break sensor wired into the intake tunnel.  When the beam is broken
# (sensor reads False), a game piece is present inside the robot.
# Assign the actual DIO port number after wiring the sensor.
INTAKE_BEAM_BREAK_PORT: int = 0

# ===========================================================================
# Drivetrain Geometry  (SDS MK4i)
# ===========================================================================

# Wheelbase / track width — both are 22 inches converted to metres.
WHEELBASE_M: float = 0.5588  # 22 in → 0.5588 m
TRACK_WIDTH_M: float = 0.5588  # 22 in → 0.5588 m

# Billet wheel diameter (4 in standard for MK4i).
WHEEL_DIAMETER_M: float = 0.1016  # 4 in → 0.1016 m
WHEEL_CIRCUMFERENCE_M: float = math.pi * WHEEL_DIAMETER_M

# ---------------------------------------------------------------------------
# SDS MK4i gear ratios (drive only).
# Select the desired ratio by assigning one of the constants below to
# DRIVE_GEAR_RATIO.  L2 is the default for this robot.
#
#   L1 = 8.14 : 1  (lower speed, higher torque)
#   L2 = 6.75 : 1  (recommended balance)
#   L3 = 6.12 : 1
#   L4 = 5.14 : 1  (highest speed)
# ---------------------------------------------------------------------------
MK4I_L1_DRIVE_RATIO: float = 8.14
MK4I_L2_DRIVE_RATIO: float = 6.75
MK4I_L3_DRIVE_RATIO: float = 6.12
MK4I_L4_DRIVE_RATIO: float = 5.14

# Active gear ratio for this robot.
DRIVE_GEAR_RATIO: float = MK4I_L2_DRIVE_RATIO

# MK4i steer (azimuth) gear ratio — fixed by SDS hardware.
STEER_GEAR_RATIO: float = 150.0 / 7.0  # ≈ 21.43 : 1

# ===========================================================================
# Robot Physical Specifications
# ===========================================================================

ROBOT_MASS_KG: float = 56.7  # 125 lbs → 56.7 kg

# ===========================================================================
# Speed Limits
# ===========================================================================

# Theoretical maximum drive speed based on Kraken X60 free speed (~6000 RPM).
KRAKEN_FREE_SPEED_RPM: float = 6000.0
KRAKEN_FREE_SPEED_RPS: float = KRAKEN_FREE_SPEED_RPM / 60.0

MAX_DRIVE_SPEED_MPS: float = KRAKEN_FREE_SPEED_RPS / DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE_M

# Practical caps applied during teleop (tune as needed).
TELEOP_MAX_SPEED_MPS: float = MAX_DRIVE_SPEED_MPS * 0.85
TELEOP_MAX_ANGULAR_SPEED_RAD_PER_S: float = math.pi * 3  # 1.5 full rotations/s

# Speed used for simple autonomous driving routines.
# 0.5 m/s is intentionally conservative — safe and easy to stop quickly.
# Raise this value to move faster during auto.
AUTO_DRIVE_SPEED_MPS: float = 0.5

# ===========================================================================
# PID / Control Gains
# ===========================================================================

# ---------------------------------------------------------------------------
# Drive motor velocity PID (Phoenix 6 slot 0, units: rotations/s)
# ---------------------------------------------------------------------------
DRIVE_KP: float = 0.1
DRIVE_KI: float = 0.0
DRIVE_KD: float = 0.0
DRIVE_KS: float = 0.1  # Static friction feed-forward (V)
DRIVE_KV: float = 0.12  # Velocity feed-forward (V / (rot/s))

# ---------------------------------------------------------------------------
# Steer motor position PID (Phoenix 6 slot 0, units: rotations)
# ---------------------------------------------------------------------------
STEER_KP: float = 100.0
STEER_KI: float = 0.0
STEER_KD: float = 0.5
STEER_KS: float = 0.0
STEER_KV: float = 0.0

# ---------------------------------------------------------------------------
# Heading correction PID (robot heading, units: radians)
# Used by SwerveDrive to maintain a target heading during straight-line driving.
# When the driver commands zero rotation (omega == 0), the drivetrain locks
# onto its current heading and uses this PID to correct any drift.
# ---------------------------------------------------------------------------
HEADING_KP: float = 5.0
HEADING_KI: float = 0.0
HEADING_KD: float = 0.0

# Minimum omega (rad/s) below which the driver is considered NOT rotating.
# If the driver inputs less than this, we activate heading hold.
# If they input more, we let them steer freely and update the locked heading.
HEADING_HOLD_OMEGA_THRESHOLD_RAD_S: float = 0.05

# ---------------------------------------------------------------------------
# PathPlanner path-following PIDs
# ---------------------------------------------------------------------------
# PathPlanner uses two PID controllers to follow a path:
#   * Translation PID — corrects X/Y position error (how far off the path we are)
#   * Rotation PID    — corrects heading error (which direction we are facing)
#
# These values are starting points.  Tune them on the real robot:
#   - Raise PP_TRANSLATION_KP if the robot is slow to correct position errors.
#   - Raise PP_ROTATION_KP if the robot is slow to correct heading errors.
#   - Add PP_TRANSLATION_KD / PP_ROTATION_KD only if the robot oscillates.
#
# Units: translation is metres, rotation is radians.
PP_TRANSLATION_KP: float = 5.0
PP_TRANSLATION_KI: float = 0.0
PP_TRANSLATION_KD: float = 0.0
PP_ROTATION_KP: float = 5.0
PP_ROTATION_KI: float = 0.0
PP_ROTATION_KD: float = 0.0

# Coefficient of friction between the drive wheels and the carpet.
# PathPlanner uses this to model wheel slip during path generation.
# 1.0 is a safe starting value for typical FRC carpet.
PP_WHEEL_COF: float = 1.0

# Moment of inertia of the robot about the vertical axis (kg * m^2).
# Calculated as approximately (1/12) * mass * (width^2 + length^2) for a
# rectangular robot.  Adjust if you have a more accurate measurement.
PP_ROBOT_MOI_KG_M2: float = (1.0 / 12.0) * ROBOT_MASS_KG * (WHEELBASE_M**2 + TRACK_WIDTH_M**2)

# ---------------------------------------------------------------------------
# Intake PID (open-loop percent output, no closed-loop needed for basic use)
# ---------------------------------------------------------------------------
INTAKE_GEAR_RATIO: float = 5.0  # 5 : 1 reduction through gearbox
INTAKE_SPEED_PERCENT: float = 0.75  # 75 % output during intaking
INTAKE_EJECT_SPEED_PERCENT: float = -0.5  # 50 % reverse during ejecting

# Velocity feed-forward used by the physics sim to model intake motor speed.
# Falcon 500 free speed ≈ 6380 RPM ≈ 106 RPS.  kV = 12 V / 106 RPS ≈ 0.113.
INTAKE_KV: float = 0.113  # V / (rot/s)

# ---------------------------------------------------------------------------
# Shooter velocity PID (Phoenix 6 slot 0, units: rotations/s)
# ---------------------------------------------------------------------------
SHOOTER_TARGET_RPM: float = 3500.0  # Reasonable flywheel speed for mid-range shot
SHOOTER_TARGET_RPS: float = SHOOTER_TARGET_RPM / 60.0
SHOOTER_AT_SPEED_TOLERANCE_RPS: float = 50.0 / 60.0  # ±50 RPM tolerance

SHOOTER_KP: float = 0.11
SHOOTER_KI: float = 0.0
SHOOTER_KD: float = 0.0
SHOOTER_KS: float = 0.1
SHOOTER_KV: float = 0.12

# ===========================================================================
# Motor Configuration
# ===========================================================================

# Current limits (Amps) — protect motors while maintaining performance.
DRIVE_STATOR_CURRENT_LIMIT_A: float = 80.0
DRIVE_SUPPLY_CURRENT_LIMIT_A: float = 40.0

STEER_STATOR_CURRENT_LIMIT_A: float = 40.0
STEER_SUPPLY_CURRENT_LIMIT_A: float = 30.0

INTAKE_STATOR_CURRENT_LIMIT_A: float = 40.0
INTAKE_SUPPLY_CURRENT_LIMIT_A: float = 30.0

SHOOTER_STATOR_CURRENT_LIMIT_A: float = 80.0
SHOOTER_SUPPLY_CURRENT_LIMIT_A: float = 40.0

# ===========================================================================
# Controller
# ===========================================================================

DRIVER_CONTROLLER_PORT: int = 0
OPERATOR_CONTROLLER_PORT: int = 1  # Reserved for future operator controller

# Deadband applied to joystick axes to eliminate drift near centre.
# Any joystick reading whose absolute value is below this is treated as 0.
# This prevents the robot from slowly drifting when the driver isn't touching
# the stick (all controllers have a little "wobble" at rest).
JOYSTICK_DEADBAND: float = 0.1

# How far a trigger must be pulled before it counts as "pressed".
# 0.0 = barely touched, 1.0 = fully pressed.  0.15 avoids accidental presses.
TRIGGER_THRESHOLD: float = 0.15

# ===========================================================================
# Vision — Limelight AprilTag Localisation
# ===========================================================================

# NetworkTables table names for each Limelight camera.
# These must match the hostnames configured in the Limelight web interface.
LIMELIGHT_FRONT_NAME: str = "limelight-front"
LIMELIGHT_REAR_NAME: str = "limelight-rear"

# Minimum number of AprilTags that must be visible before a pose estimate
# is trusted enough to fuse into odometry.
LIMELIGHT_MIN_TAGS_FOR_FUSION: int = 1

# Maximum ambiguity ratio for a single-tag pose to be accepted.
# Lower = more confident.  Range 0.0–1.0; reject if above this threshold.
LIMELIGHT_MAX_AMBIGUITY: float = 0.2

# Maximum distance (metres) from the estimated vision pose to the current
# odometry pose before the vision measurement is rejected as an outlier.
LIMELIGHT_MAX_POSE_JUMP_M: float = 1.0

# Standard deviations for the vision pose measurement noise model.
# Larger values = trust vision less relative to wheel odometry.
# Format: (x_m, y_m, heading_rad)
LIMELIGHT_SINGLE_TAG_STD_DEVS: tuple[float, float, float] = (0.9, 0.9, float("inf"))
LIMELIGHT_MULTI_TAG_STD_DEVS: tuple[float, float, float] = (0.3, 0.3, float("inf"))

# Physical camera mount positions on the robot (metres, radians).
# Measured from robot centre; +x forward, +y left, CCW positive rotation.
# Adjust after physically mounting the cameras.
LIMELIGHT_FRONT_POSE: tuple[float, float, float] = (0.25, 0.0, 0.0)  # x, y, yaw_rad
LIMELIGHT_REAR_POSE: tuple[float, float, float] = (-0.25, 0.0, 3.14159)  # facing rear

# ===========================================================================
# System Identification (SysId) — Drive Characterisation
# ===========================================================================

# Mechanism name used as a prefix in DataLog entries and the SysId tool.
SYSID_MECHANISM_NAME: str = "Drive"

# Quasistatic test: voltage ramp rate.  The robot slowly accelerates so the
# system can measure velocity vs voltage in near-steady-state.
# Typical range: 0.1 – 0.5 V/s.  Start conservative.
SYSID_QUASISTATIC_RAMP_RATE_V_PER_S: float = 0.25

# Dynamic test: step voltage applied instantly to get the system's dynamic
# response.  Typical range: 3 – 7 V.
SYSID_DYNAMIC_STEP_VOLTAGE_V: float = 4.0

# Maximum voltage ever applied during a SysId test (safety cap).
SYSID_MAX_VOLTAGE_V: float = 7.0

# ===========================================================================
# Phoenix 6 Motor Controller Configuration
# ===========================================================================

# Phoenix 6 supports up to 10 independent PID slots per motor controller.
# We put the primary PID gains in slot 0.  This number tells the motor
# "use the gains stored in slot 0 for this control request."
DRIVE_PID_SLOT: int = 0
STEER_PID_SLOT: int = 0

# The steer motor is directly geared to the module's steering axle, so the
# encoder reading equals the actual wheel angle — ratio of 1:1.
STEER_SENSOR_TO_MECHANISM_RATIO: float = 1.0

# The CANcoder reports positions in the range [-0.5, +0.5] rotations.
# Setting the "discontinuity point" to 0.5 means the wrap-around happens
# exactly at the halfway point (±180°), which is the most natural choice.
CANCODER_DISCONTINUITY_POINT: float = 0.5

# ===========================================================================
# Limelight NetworkTables Array Indices
# ===========================================================================
# The Limelight publishes the robot pose as a 11-element array called
# "botpose_wpiblue".  These named indices make the code readable — instead
# of writing pose_array[7] you write pose_array[LIMELIGHT_BOTPOSE_TAG_COUNT_IDX].
#
# Full array layout from the Limelight documentation:
#   [0] x_m          — robot X position on the field (metres)
#   [1] y_m          — robot Y position on the field (metres)
#   [2] z_m          — robot height above the floor (metres, usually ignored)
#   [3] roll_deg     — roll angle (usually ignored for ground robots)
#   [4] pitch_deg    — pitch angle (usually ignored for ground robots)
#   [5] yaw_deg      — robot heading on the field (degrees)
#   [6] latency_ms   — total vision processing latency (milliseconds)
#   [7] tag_count    — number of AprilTags visible in this frame
#   [8] tag_span_m   — distance between the outermost detected tags
#   [9] avg_dist_m   — average distance from robot to all visible tags
#  [10] avg_area_pct — average tag area as % of camera image (0–100)
LIMELIGHT_BOTPOSE_NUM_POSE_ELEMENTS: int = 6  # indices 0-5 are the 6D pose
LIMELIGHT_BOTPOSE_TAG_COUNT_IDX: int = 7
LIMELIGHT_BOTPOSE_AVG_AREA_IDX: int = 10
LIMELIGHT_BOTPOSE_ARRAY_MIN_LEN: int = 8  # minimum length to read tag count
LIMELIGHT_BOTPOSE_FULL_LEN: int = 11  # full array length

# Limelight LED mode values (written to the "ledMode" NetworkTables key).
# These come from the Limelight documentation.
LIMELIGHT_LED_PIPELINE: float = 0.0  # Let the active pipeline control LEDs
LIMELIGHT_LED_OFF: float = 1.0  # Force LEDs off
LIMELIGHT_LED_BLINK: float = 2.0  # Force LEDs to blink
LIMELIGHT_LED_ON: float = 3.0  # Force LEDs on

# When checking tag area as an ambiguity proxy (single-tag only), we need a
# minimum area threshold.  This formula converts the LIMELIGHT_MAX_AMBIGUITY
# constant into an area-percent floor.
# A tag that fills less than this % of the image is considered too far away
# or at too steep an angle to trust for single-tag localisation.
LIMELIGHT_MIN_AREA_FOR_SINGLE_TAG: float = (1.0 - LIMELIGHT_MAX_AMBIGUITY) * 10.0

# ===========================================================================
# Field Geometry — 2026 FRC (Reefscape)
# ===========================================================================
# WPILib field-coordinate origin is the blue-alliance corner (bottom-left
# when viewed from above with blue on the left).  X increases toward the
# red-alliance wall; Y increases toward the top of the field.
#
#   Blue wall   Wing line           Centre         Wing line   Red wall
#   X = 0       X = BLUE_ZONE_END   X = FIELD/2    X = RED_ZONE_START   X = FIELD_LENGTH_M
#   |           |                   |              |           |
#   |<—— BLUE ——>|<————— NEUTRAL ————————————————>|<—— RED ——>|
#
# Field dimensions (Reefscape 2025 / 2026 field, converted to metres):
#   Length: 690.875 in  = 17.548 m
#   Width:  317.000 in  =  8.052 m
FIELD_LENGTH_M: float = 17.548  # X extent of the full field
FIELD_WIDTH_M: float = 8.052  # Y extent of the full field

# X-coordinate boundaries that separate alliance zones from the neutral zone.
# The wing lines in Reefscape are 10 ft 5.875 in (≈ 3.20 m) from each wall.
# Anything inside those lines is considered that alliance's "side."
BLUE_ZONE_END_M: float = 5.18  # Blue alliance side:  0 ≤ X < BLUE_ZONE_END_M
RED_ZONE_START_M: float = 12.37  # Red alliance side:   RED_ZONE_START_M < X ≤ FIELD_LENGTH_M
# Neutral zone: BLUE_ZONE_END_M ≤ X ≤ RED_ZONE_START_M

# ===========================================================================
# AprilTag Field Layout — 2026 FRC (Reefscape Rebuilt)
# ===========================================================================
# Change APRILTAG_FIELD to switch between the two 2026 field variants:
#
#   k2026RebuiltWelded    — standard welded-steel field (most events)
#   k2026RebuiltAndyMark  — AndyMark field panels (some events use these)
#
# The 2026 Reefscape Rebuilt (Welded) field has 32 AprilTags:
#   IDs  1-12 : Red reef cluster    (x ≈ 11.3–12.5 m)
#   IDs 13-16 : Red barge wall      (x ≈ 16.5 m)
#   IDs 17-28 : Blue reef cluster   (x ≈ 4.0–5.2 m)
#   IDs 29-32 : Blue barge wall     (x ≈ 0.0 m)
#
# This enum value is read by FieldState.setup() to load the layout once
# at robot init time.  No other code needs to import robotpy_apriltag.
from robotpy_apriltag import AprilTagField as _AprilTagField  # noqa: E402

# ← Change this one line to switch field variants at an event.
APRILTAG_FIELD: _AprilTagField = _AprilTagField.k2026RebuiltWelded

# Distance (metres) within which a tag is considered "nearby" — i.e. the
# robot is close enough that a Limelight should realistically be able to
# see it.  Tags outside this radius are unexpected; seeing them suggests
# a false positive or a badly drifted odometry estimate.
# 4 m covers the typical scoring approach distance.
APRILTAG_PROXIMITY_RADIUS_M: float = 4.0

# When the Limelight sees a tag that matches an expected nearby tag, we
# reward it with tighter standard deviations (more trust in the vision fix).
# Format: (x_m, y_m, heading_rad) — heading stays "inf" because MegaTag2
# yaw is driven by the IMU, not tag geometry.
LIMELIGHT_CONFIRMED_TAG_STD_DEVS: tuple[float, float, float] = (0.15, 0.15, float("inf"))

# ===========================================================================
# LED Subsystem — REV Blinkin LED Driver (PWM)
# ===========================================================================
# The Blinkin receives a PWM signal on a standard servo port.
# Values from 0.0–1.0 select different colour/pattern presets.
# Full pattern table: https://www.revrobotics.com/content/docs/REV-11-1105-UD.pdf
BLINKIN_PWM_PORT: int = 0  # roboRIO PWM header number

# Pattern values (0.0–1.0 range) — from the REV Blinkin pattern table.
# Adjust these after consulting the Blinkin documentation for your firmware.
BLINKIN_IDLE: float = 0.77  # Solid white  — robot ready, no piece
BLINKIN_INTAKING: float = 0.65  # Strobe white — intake running
BLINKIN_HOLDING: float = 0.69  # Solid green  — game piece held
BLINKIN_SPINNING_UP: float = 0.61  # Fast strobe yellow — flywheel spinning up
BLINKIN_READY_TO_FIRE: float = 0.73  # Solid orange — at speed, ready to fire
BLINKIN_FIRING: float = 0.59  # Strobe red   — firing sequence active

# ===========================================================================
# Loop Overrun Watchdog
# ===========================================================================
# Alert threshold: if any robot loop takes longer than this many milliseconds,
# a warning is published to the dashboard.  The WPILib default loop period
# is 20 ms; we alert at 75 % of that budget to catch consistent overruns.
LOOP_OVERRUN_THRESHOLD_MS: float = 15.0  # ms

# ===========================================================================
# Operator Controller
# ===========================================================================
# Second Xbox controller for an operator (shooter speed tuning, overrides, etc.)
# Declared here but wired in robot.py.  Set port to -1 to disable.
OPERATOR_CONTROLLER_PORT: int = 1

# Shooter speed presets the operator can select (rotations/second).
# These are alternative flywheel targets — e.g. short shot vs. long shot.
SHOOTER_SPEED_SHORT_RPS: float = 40.0  # ≈ 2400 RPM
SHOOTER_SPEED_LONG_RPS: float = 70.0  # ≈ 4200 RPM
