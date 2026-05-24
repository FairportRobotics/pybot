"""Main robot entry point.

``MyRobot`` is a ``MagicRobot`` subclass.  MagicBot automatically:
  - Instantiates all components declared as class-level type annotations.
  - Calls each component's ``execute()`` method every robot loop.
  - Injects class-level variables into components that share the same name.

Driver control scheme (Xbox controller, port 0)
------------------------------------------------
| Input              | Action                                  |
|--------------------|------------------------------------------|
| Left stick         | Translation (vx / vy)                   |
| Right stick X      | Rotation (omega)                        |
| Left trigger       | Run intake (scoring state machine)      |
| Right bumper       | Fire game piece (if held + at speed)    |
| Left bumper        | Eject (reverse intake, manual override) |
| Right trigger      | Manual shooter spin-up override         |
| X button           | Reset gyro heading                      |
| Y button           | Toggle field-relative / robot-relative  |
| A button           | (reserved)                              |
| B button           | (reserved)                              |
| Start button       | (reserved)                              |
| Back button        | (reserved)                              |
| D-Pad              | (reserved)                              |

Operator control scheme (Xbox controller, port 1)
--------------------------------------------------
| Input              | Action                                  |
|--------------------|------------------------------------------|
| A button           | Select short shot speed                 |
| Y button           | Select long shot speed                  |
| B button           | Reset shooter speed to default          |

Test mode — System Identification (SysId)
-----------------------------------------
Enable Test mode on the Driver Station, then HOLD a button.
Release to stop. Data is logged to a .wpilog file.

| Button | SysId Routine              |
|--------|----------------------------|
| A      | Quasistatic Forward        |
| B      | Quasistatic Reverse        |
| X      | Dynamic (step) Forward     |
| Y      | Dynamic (step) Reverse     |

Copy the .wpilog from the robot (or sim working dir) and open it in the
WPILib SysId Analysis tool to extract kS, kV, kA, kP gains.

Vision / Localisation
---------------------
Two Limelight 3 cameras (front + rear) run continuously in all modes.
Their AprilTag pose estimates are fused into the SwerveDrive pose estimator
when quality gates (tag count, ambiguity, pose-jump distance) are met.
The AprilTag field layout (2026 Reefscape Rebuilt) is used to validate
which tags the robot should be seeing based on odometry, improving
localisation confidence.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import wpilib
from magicbot import feedback
from wpilib import DriverStation

import constants
import state_machine as sm
from components.field_state import FieldState
from components.intake import Intake
from components.leds import LEDSubsystem
from components.limelight import Limelight
from components.mechanism_display import MechanismDisplay
from components.shooter import Shooter
from components.swerve_drive import SwerveDrive
from components.sysid import DrivetrainSysId
from components.xbox_controller import XboxController
from genie import GenieRobot


class MyRobot(GenieRobot):
    """Top-level MagicRobot class.

    Component declarations
    ----------------------
    MagicBot reads the type annotations below and injects fully-constructed
    component instances into this class.  Class-level variables with the same
    name prefix are injected *into* the component as well (e.g.
    ``driver_controller_invert_axes`` → ``XboxController.invert_axes``).
    """

    # --- Components (MagicBot constructs and wires these automatically) ---
    swerve_drive: SwerveDrive
    intake: Intake
    shooter: Shooter
    sysid: DrivetrainSysId
    scoring: sm.ScoringStateMachine
    field_state: FieldState
    leds: LEDSubsystem
    mechanism_display: MechanismDisplay
    # Two Limelight cameras — MagicBot creates one instance per annotation.
    # The ``table_name`` and ``swerve_drive`` attributes are injected by
    # MagicBot using the name-prefix convention:
    #   limelight_front_table_name → limelight_front.table_name
    #   limelight_front_swerve_drive is satisfied by the shared swerve_drive
    limelight_front: Limelight
    limelight_rear: Limelight

    # --- MagicBot injection: Limelight NetworkTables table names ---
    limelight_front_table_name: str = constants.LIMELIGHT_FRONT_NAME
    limelight_rear_table_name: str = constants.LIMELIGHT_REAR_NAME

    # --- MagicBot injection: passed into XboxController.invert_axes ---
    # Set False to disable axis inversion (useful on some controller firmware).
    driver_controller_invert_axes: bool = True
    # --- MagicBot injection: passed into XboxController.port ---
    driver_controller_port: int = constants.DRIVER_CONTROLLER_PORT

    # ------------------------------------------------------------------
    # Git version
    # ------------------------------------------------------------------

    @staticmethod
    def _read_git_hash() -> str:
        """Return the git commit hash for the deployed code.

        Resolution order:
        1. ``version.txt`` in the project root — written by
           ``write_version.py`` before ``robotpy deploy``.  This is the
           only source available on the roboRIO (git is not on its PATH).
        2. ``subprocess`` call to ``git rev-parse --short HEAD`` — works on
           the developer machine during simulation.
        3. ``"unknown"`` — fallback when neither source is available.
        """
        version_file = Path(__file__).parent / "version.txt"
        if version_file.exists():
            return version_file.read_text(encoding="utf-8").strip()

        try:
            short_hash = subprocess.check_output(
                ["git", "rev-parse", "--short", "HEAD"],
                stderr=subprocess.DEVNULL,
                text=True,
            ).strip()
            dirty = subprocess.call(["git", "diff", "--quiet", "HEAD"], stderr=subprocess.DEVNULL)
            return f"{short_hash}-dirty" if dirty else short_hash
        except (subprocess.CalledProcessError, FileNotFoundError, OSError):
            return "unknown"

    @feedback
    def git_hash(self) -> str:
        """Git commit hash of the deployed code (e.g. ``3ca6270-dirty``).

        Visible on any dashboard that reads NetworkTables, including
        AdvantageScope's metadata panel.  Populated from ``version.txt``
        (written by ``write_version.py`` before ``robotpy deploy``) so
        it works on the roboRIO where git is not installed.
        """
        return self._git_hash

    # ------------------------------------------------------------------
    # MagicBot lifecycle
    # ------------------------------------------------------------------

    def createObjects(self) -> None:
        """Instantiate any objects not managed by MagicBot injection.

        ``XboxController`` instances are pure input helpers (no ``execute()``
        loop), so they are created here rather than declared as MagicBot
        components.

        Note: PathPlanner's ``AutoBuilder.configure()`` is called in
        ``robotInit()`` (below), not here.  Components are not yet injected
        when ``createObjects()`` runs, so ``self.swerve_drive`` would not
        exist yet.  ``super().robotInit()`` must finish first.
        """
        self.driver_controller = XboxController(
            port=self.driver_controller_port,
            invert_axes=self.driver_controller_invert_axes,
        )
        # Operator controller — handles shooter speed selection and manual overrides.
        # Disabled by default (port from constants); swap port to -1 to skip.
        self.operator_controller = XboxController(
            port=constants.OPERATOR_CONTROLLER_PORT,
            invert_axes=False,
        )

    def robotInit(self) -> None:
        """Full robot initialisation.

        MagicBot's ``robotInit()`` (called via ``super()``) runs
        ``createObjects()`` and then creates + injects all components.
        Once ``super().robotInit()`` returns, every component annotation
        (``swerve_drive``, ``intake``, etc.) is a fully constructed object
        and we can safely configure PathPlanner.
        """
        super().robotInit()

        # --- Git commit hash ---
        # Read from version.txt (written by write_version.py before deploy).
        # Falls back to subprocess in sim (where git is available on the dev
        # machine), then to "unknown" if neither works.
        self._git_hash: str = self._read_git_hash()

        # --- Data logging ---
        # Controlled by LOGGING_ENABLED in constants.py.
        # Writes a timestamped .wpilog to /home/lvuser/logs/ on the robot
        # and to the working directory in simulation.  Captures all NT keys.
        if constants.LOGGING_ENABLED:
            wpilib.DataLogManager.start()
            wpilib.DataLogManager.logNetworkTables(True)

        # --- Loop overrun watchdog ---
        # Log a warning to the dashboard if any loop takes longer than the
        # threshold.  The Watchdog resets automatically at the start of each
        # loop; just call addEpoch() after expensive operations to pinpoint
        # where time is being spent.
        self._watchdog = wpilib.Watchdog(
            constants.LOOP_OVERRUN_THRESHOLD_MS / 1000.0,
            lambda: wpilib.reportWarning(
                f"Loop overrun! Period exceeded {constants.LOOP_OVERRUN_THRESHOLD_MS} ms"
            ),
        )

        self._configure_pathplanner()

    def _configure_pathplanner(self) -> None:
        """Register the drivetrain with PathPlanner's AutoBuilder.

        This must be called after MagicBot has injected all components so
        that ``self.swerve_drive`` exists.  Pulling it into its own method
        keeps ``robotInit()`` readable.
        """
        from pathplannerlib.auto import AutoBuilder
        from pathplannerlib.config import ModuleConfig, PIDConstants, RobotConfig
        from pathplannerlib.controller import PPHolonomicDriveController
        from wpimath.geometry import Translation2d
        from wpimath.system.plant import DCMotor

        # Describe one swerve module to PathPlanner so it can model the
        # robot's acceleration limits and path-optimisation constraints.
        half_wb = constants.WHEELBASE_M / 2.0
        half_tw = constants.TRACK_WIDTH_M / 2.0
        module_config = ModuleConfig(
            wheelRadiusMeters=constants.WHEEL_DIAMETER_M / 2.0,
            maxDriveVelocityMPS=constants.MAX_DRIVE_SPEED_MPS,
            wheelCOF=constants.PP_WHEEL_COF,
            # Kraken X60 with FOC, reduced by the drive gear ratio.
            driveMotor=DCMotor.krakenX60FOC().withReduction(constants.DRIVE_GEAR_RATIO),
            driveCurrentLimit=constants.DRIVE_STATOR_CURRENT_LIMIT_A,
            numMotors=1,
        )
        robot_config = RobotConfig(
            massKG=constants.ROBOT_MASS_KG,
            MOI=constants.PP_ROBOT_MOI_KG_M2,
            moduleConfig=module_config,
            moduleOffsets=[
                Translation2d(half_wb, half_tw),  # Front-Left
                Translation2d(half_wb, -half_tw),  # Front-Right
                Translation2d(-half_wb, half_tw),  # Back-Left
                Translation2d(-half_wb, -half_tw),  # Back-Right
            ],
        )

        AutoBuilder.configure(
            # Where is the robot right now?
            pose_supplier=self.swerve_drive.get_pose,
            # Snap the pose estimator to a known pose (used at auto start).
            reset_pose=self.swerve_drive.reset_pose,
            # How fast is the robot actually moving (from encoders)?
            robot_relative_speeds_supplier=self.swerve_drive.get_chassis_speeds,
            # Send PathPlanner's speed command to the drivetrain.
            # The second argument is DriveFeedforwards — ignored here since
            # Phoenix 6 closed-loop handles feedforward internally.
            output=lambda speeds, _ff: self.swerve_drive.drive_chassis_speeds(speeds),
            # PID controllers that correct for path-following error.
            controller=PPHolonomicDriveController(
                PIDConstants(
                    constants.PP_TRANSLATION_KP,
                    constants.PP_TRANSLATION_KI,
                    constants.PP_TRANSLATION_KD,
                ),
                PIDConstants(
                    constants.PP_ROTATION_KP,
                    constants.PP_ROTATION_KI,
                    constants.PP_ROTATION_KD,
                ),
            ),
            robot_config=robot_config,
            # Flip paths to the red side when we are on the red alliance.
            # PathPlanner paths are always drawn from the blue alliance origin.
            should_flip_path=lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed,
            # MagicBot has no Subsystem objects — pass None.
            drive_subsystem=None,
        )

    # ------------------------------------------------------------------
    # Teleop
    # ------------------------------------------------------------------

    def teleopInit(self) -> None:
        """Called once when teleop begins.

        A good place to reset any transient state (e.g. ensure the robot
        starts in robot-relative mode regardless of how autonomous left it).
        """
        self.swerve_drive.set_field_relative(False)

    def teleopPeriodic(self) -> None:
        """Called every ~20 ms loop during teleop.

        Read the driver and operator controllers and translate inputs into
        component method calls.  MagicBot calls ``execute()`` on each
        component *after* this method returns.
        """
        self._watchdog.reset()  # restart overrun timer each loop

        ctrl = self.driver_controller
        op = self.operator_controller

        # --- Driving ---
        self.swerve_drive.drive(
            vx=ctrl.vx * constants.TELEOP_MAX_SPEED_MPS,
            vy=ctrl.vy * constants.TELEOP_MAX_SPEED_MPS,
            omega=ctrl.omega * constants.TELEOP_MAX_ANGULAR_SPEED_RAD_PER_S,
        )
        self._watchdog.addEpoch("drive")

        # --- Drivetrain mode toggles ---
        if ctrl.y_button_pressed:
            self.swerve_drive.toggle_field_relative()

        if ctrl.x_button_pressed:
            self.swerve_drive.reset_gyro()

        # --- Intake / Shooter (state machine) ---
        if ctrl.left_trigger > constants.TRIGGER_THRESHOLD:
            self.scoring.request_intake()
        elif ctrl.left_bumper:
            self.intake.eject()

        if ctrl.right_bumper:
            self.scoring.request_fire()

        # Manual shooter spin-up override (right trigger)
        if ctrl.right_trigger > constants.TRIGGER_THRESHOLD:
            self.shooter.spin_up()
        elif not self.scoring.is_executing:
            self.shooter.stop()

        # --- Operator controller — shooter speed presets ---
        if op.a_button_pressed:
            self.shooter.set_speed(constants.SHOOTER_SPEED_SHORT_RPS)
        elif op.y_button_pressed:
            self.shooter.set_speed(constants.SHOOTER_SPEED_LONG_RPS)
        elif op.b_button_pressed:
            self.shooter.reset_speed()

        self._watchdog.addEpoch("controls")

    # ------------------------------------------------------------------
    # Test mode — System Identification
    # ------------------------------------------------------------------

    def testInit(self) -> None:
        """Called once when Test mode begins.

        DataLogManager is already started in ``robotInit()`` when
        ``LOGGING_ENABLED`` is True, so SysId data is always captured.
        We just ensure field-relative is disabled for clean characterisation.
        """
        if not constants.LOGGING_ENABLED:
            # Start logging now if it wasn't started at init.
            wpilib.DataLogManager.start()
        self.swerve_drive.set_field_relative(False)

    def testPeriodic(self) -> None:
        """Called every ~20 ms loop while in Test mode.

        Translate Xbox controller buttons into SysId routine commands.
        Hold a button to run the routine; release to stop.

        | Button | Routine                    |
        |--------|----------------------------|
        | A      | Quasistatic Forward        |
        | B      | Quasistatic Reverse        |
        | X      | Dynamic (step) Forward     |
        | Y      | Dynamic (step) Reverse     |
        """
        ctrl = self.driver_controller

        if ctrl.a_button:
            self.sysid.quasistatic_forward()
        elif ctrl.b_button:
            self.sysid.quasistatic_reverse()
        elif ctrl.x_button:
            self.sysid.dynamic_forward()
        elif ctrl.y_button:
            self.sysid.dynamic_reverse()

    # ------------------------------------------------------------------
    # Disabled
    # ------------------------------------------------------------------

    def disabledPeriodic(self) -> None:
        """Called every loop while the robot is disabled.

        The Limelight components continue executing (MagicBot calls all
        component ``execute()`` methods in every mode), so vision-based
        pose seeding happens automatically before the match starts.
        This is intentional — the pose estimator will converge to the
        correct starting pose while the robot sits on the field.
        """
        pass
