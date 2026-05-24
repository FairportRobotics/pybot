# pybot — FRC Swerve Drive Robot (RobotPy / MagicBot)

A Python-based FRC robot codebase for a four-wheel swerve drive robot built on
[RobotPy](https://robotpy.readthedocs.io/) and the
[MagicBot](https://robotpy.readthedocs.io/projects/utilities/en/stable/magicbot.html)
framework. Designed for the **2026 Reefscape Rebuilt** game season.

---

## Hardware

| Subsystem | Hardware |
|---|---|
| Drivetrain | SDS MK4i swerve modules (L2 gear ratio, 6.75:1) |
| Drive motors | CTRE Kraken X60 (FOC) — 4× |
| Steer motors | CTRE Kraken X60 — 4× |
| Steer encoders | CTRE CANcoder — 4× |
| Gyro | CTRE Pigeon 2 |
| CAN bus | CANivore (`"canivore"`) |
| Intake motor | VEX Falcon 500 (TalonFX), 5:1 gearbox |
| Shooter motor | VEX Falcon 500 (TalonFX), flywheel |
| Vision | Limelight 3 — 2× (front + rear) |
| LED controller | REV Blinkin (PWM) |
| Controllers | 2× Xbox (driver port 0, operator port 1) |

---

## Software Stack

| Package | Version |
|---|---|
| Python | 3.12.1 |
| RobotPy | 2026.2.2 |
| MagicBot | latest (via RobotPy) |
| Phoenix 6 (standalone) | 26.2.0 |
| robotpy-pathplannerlib | 2026.1.2 |
| robotpy-apriltag | 2026.2.2 |
| ruff | 0.15.14 |
| pytest / pyfrc | 9.0.3 / 2026.0.2 |

> **Note:** The standalone `phoenix6` pip package is used instead of
> `robotpy-phoenix6`. Including `robotpy-phoenix6` would load a HALSim
> extension that conflicts with the standalone package's C-level thread and
> crashes `robotpy sim`.

---

## Repository Layout

```
pybot/
├── robot.py                  # Top-level MagicRobot entry point
├── constants.py              # All tuning parameters and CAN IDs
├── physics.py                # pyfrc physics engine (sim only)
├── write_version.py          # Writes git hash to version.txt before deploy
├── version.txt               # Short git hash baked in at deploy time
├── pyproject.toml            # RobotPy + ruff configuration
├── genie/                    # Custom MagicRobot base class (GenieRobot)
├── components/
│   ├── swerve_drive.py       # SwerveDrive — 4-module drivetrain + odometry
│   ├── swerve_module.py      # SwerveModule — single drive+steer module
│   ├── intake.py             # Intake — Falcon 500, beam break sensor
│   ├── shooter.py            # Shooter — Falcon 500 flywheel, velocity PID
│   ├── limelight.py          # Limelight — AprilTag vision fusion
│   ├── leds.py               # LEDSubsystem — REV Blinkin driver
│   ├── field_state.py        # FieldState — zone tracking + tag validation
│   ├── mechanism_display.py  # MechanismDisplay — Mechanism2d widget
│   ├── sysid.py              # DrivetrainSysId — WPILib SysId routines
│   ├── xbox_controller.py    # XboxController — deadband + axis helpers
│   ├── sim_devices.py        # Simulation stubs for Phoenix 6 hardware
│   └── tank_drive.py         # TankDrive — reference differential drive
├── state_machine/
│   └── scoring.py            # ScoringStateMachine — intake-to-fire sequence
└── tests/
    └── pyfrc_test.py         # pytest / pyfrc smoke tests
```

---

## Components

### `SwerveDrive`
- Owns four `SwerveModule` instances (FL, FR, BL, BR) and the Pigeon 2 gyro.
- Runs `SwerveDrive4Kinematics` to convert chassis speeds to per-module states.
- Maintains field pose via `SwerveDrive4PoseEstimator`, fusing wheel odometry
  with Limelight vision measurements.
- Supports **field-relative** and **robot-relative** driving modes.
- Publishes odometry pose, heading, speed, and mode to NetworkTables via
  `@feedback` for dashboard display.
- **Heading hold:** When the driver's rotation stick is below threshold, the
  drivetrain locks its heading and uses a PID loop to resist drift.
- Exposes a shared `Field2d` widget for field overlay visualisation.

### `SwerveModule`
- Wraps one drive TalonFX, one steer TalonFX, and one CANcoder.
- Phoenix 6 closed-loop velocity control on the drive motor (slot 0).
- Phoenix 6 closed-loop position control on the steer motor (slot 0).
- CANcoder configured as fused absolute feedback for steer position.
- Module steer optimisation (`SwerveModuleState.optimize`) reduces wheel
  rotation to at most 90° when reversing drive direction.

### `Intake`
- Single Falcon 500 at 75% duty cycle with a 5:1 gearbox.
- Beam break sensor (DIO) detects a game piece in the tunnel.
- State machine: `STOPPED → INTAKING → HOLDING → EJECTING`.
- **Shooter interlock:** Intake is silently blocked if the shooter is spinning
  but has not yet reached target speed (prevents jamming).

### `Shooter`
- Single Falcon 500 flywheel in velocity closed-loop (Phoenix 6, slot 0).
- Default target: **3 500 RPM** (configurable in `constants.py`).
- `is_at_speed()` returns `True` when within ±50 RPM of target.
- Operator controller can select **short shot** / **long shot** speed presets,
  or reset to the default.

### `ScoringStateMachine`
- MagicBot `StateMachine` that sequences the intake-to-fire workflow.
- `request_intake()` → spins intake and waits for beam break.
- `request_fire()` → spins shooter, waits for `is_at_speed()`, then feeds.

### `Limelight` (×2 — front and rear)
- Reads `botpose_wpiblue` (MegaTag 2) from the Limelight NT table each loop.
- Quality gates before fusing into odometry:
  - Tag count ≥ 1
  - Single-tag: average tag area above minimum threshold
  - Pose jump: estimated pose must be within a maximum distance of current odometry
- **Odometry-consistency gate:** Cross-checks the detected AprilTag ID against
  `FieldState` to verify the robot should actually be able to see that tag.
  Confirmed detections use tighter standard deviations (more trusted).
- Latency-compensated timestamps for accurate pose estimator fusion.

### `FieldState`
- Tracks which field zone the robot is in and which AprilTag IDs are expected
  to be visible from that zone (used by the Limelight odometry gate).

### `LEDSubsystem`
- Commands a REV Blinkin LED driver via PWM based on robot state:

  | Priority | State | Pattern |
  |---|---|---|
  | 1 | Firing | Strobe red |
  | 2 | Ready (piece + at speed) | Solid orange |
  | 3 | Holding piece | Solid green |
  | 4 | Spinning up | Fast strobe yellow |
  | 5 | Intaking | Strobe white |
  | 6 | Idle | Solid white |

### `DrivetrainSysId`
- Implements WPILib's four SysId routines (quasistatic/dynamic, fwd/rev).
- Open-loop voltage is logged with position and velocity for analysis.
- Activated in **Test mode** only; releasing a button stops immediately.

---

## Control Scheme

### Driver (Xbox — port 0)

| Input | Action |
|---|---|
| Left stick | Translation (vx / vy) |
| Right stick X | Rotation (omega) |
| Left trigger | Run intake (state machine) |
| Right bumper | Fire game piece |
| Left bumper | Eject (manual reverse) |
| Right trigger | Manual shooter spin-up |
| X button | Reset gyro heading |
| Y button | Toggle field-relative / robot-relative |

### Operator (Xbox — port 1)

| Input | Action |
|---|---|
| A button | Short shot speed preset |
| Y button | Long shot speed preset |
| B button | Reset shooter speed to default |

### Test Mode — SysId (Driver controller)

| Button | Routine |
|---|---|
| A | Quasistatic Forward |
| B | Quasistatic Reverse |
| X | Dynamic Forward |
| Y | Dynamic Reverse |

Hold the button to run; release to stop. Data is logged to a `.wpilog` file.

---

## Autonomous

PathPlanner is used for path-following autonomous routines.
`AutoBuilder.configure()` registers the swerve drivetrain with PathPlanner's
holonomic drive controller.

Available autonomous modes (selectable via SmartDashboard):

| Mode | Description |
|---|---|
| **Do Nothing** *(default)* | Sits still |
| **Move Forward** | Drives straight forward at `AUTO_DRIVE_SPEED_MPS` |
| **PathPlanner Auto** | Executes a PathPlanner path from the deploy directory |

Paths are automatically mirrored to the red alliance side based on
`DriverStation.getAlliance()`.

---

## Development Setup

### Prerequisites

- Python 3.12
- Git

### Install

```powershell
python -m venv .venv
.venv\Scripts\Activate.ps1      # PowerShell
pip install -r requirements.txt  # or: pip install robotpy[...] phoenix6 ...
```

> On Windows PowerShell you may need to allow script execution first:
> ```powershell
> Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope Process
> ```

### Run the Simulator

```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope Process
.venv\Scripts\Activate.ps1
python -m robotpy sim --nogui
```

> **Note:** The `--nogui` flag suppresses the Tkinter sim window. Remove it
> if you want the graphical joystick/DS panel.

### Run Tests

```powershell
python -m robotpy test
```

Expected output: `4 passed`.

### Lint & Format

```powershell
ruff check .          # lint
ruff format --check . # format check (no changes)
ruff format .         # auto-format
```

---

## Deploy to Robot

1. **Write the version file** (bakes the git hash into the deploy bundle):

   ```powershell
   python write_version.py
   ```

2. **Deploy:**

   ```powershell
   python -m robotpy deploy
   ```

   RobotPy will sync all Python files and pip-install dependencies on the
   roboRIO automatically.

3. **Verify the git hash** on any NT-connected dashboard under
   `SmartDashboard/MyRobot/git_hash`. A `-dirty` suffix means uncommitted
   changes were present at deploy time.

---

## Data Logging

When `LOGGING_ENABLED = True` in `constants.py` (the default), WPILib's
`DataLogManager` writes all NetworkTables keys and custom log entries to a
timestamped `.wpilog` file:

- **On the robot:** `/home/lvuser/logs/`
- **In simulation:** project working directory (`logs/`)

Open `.wpilog` files in [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope)
or the WPILib Data Log Tool.

For SysId analysis, copy the `.wpilog` to your development machine and open it
in **WPILib SysId Analysis** (`Tools → SysId` in the WPILib VS Code extension).

---

## Simulation Notes

- Phoenix 6 hardware objects (`TalonFX`, `CANcoder`, `Pigeon2`) are replaced
  by lightweight Python stubs (`components/sim_devices.py`) in simulation.
  The stubs implement the same interface and return plausible sensor values.
- The `physics.py` engine integrates a simple linear motor model
  (`velocity = voltage / kV`) to drive the stubs each 20 ms tick.
- **Windows:** Use **PowerShell** (not Git Bash) to run the simulator.
  The ntcore native thread causes an access violation in the Git Bash
  environment on Windows with RobotPy 2026.

---

## Constants Reference

All tunable parameters live in `constants.py`. Key sections:

| Section | Notable constants |
|---|---|
| CAN IDs | `DRIVE_MOTOR_IDS`, `STEER_MOTOR_IDS`, `CANCODER_IDS`, `PIGEON_ID` |
| Geometry | `WHEELBASE_M`, `TRACK_WIDTH_M`, `WHEEL_DIAMETER_M` |
| Gear ratios | `DRIVE_GEAR_RATIO` (MK4i L2 = 6.75), `STEER_GEAR_RATIO` |
| Speed limits | `TELEOP_MAX_SPEED_MPS`, `TELEOP_MAX_ANGULAR_SPEED_RAD_PER_S` |
| Drive PID | `DRIVE_KP/KI/KD/KS/KV` |
| Steer PID | `STEER_KP/KI/KD` |
| Heading hold | `HEADING_KP`, `HEADING_HOLD_OMEGA_THRESHOLD_RAD_S` |
| PathPlanner | `PP_TRANSLATION_KP`, `PP_ROTATION_KP`, `PP_WHEEL_COF` |
| Shooter | `SHOOTER_TARGET_RPM`, `SHOOTER_KP/KV` |
| Intake | `INTAKE_SPEED_PERCENT`, `INTAKE_GEAR_RATIO` |
| Limelight | `LIMELIGHT_FRONT_NAME`, `LIMELIGHT_REAR_NAME`, std dev tuning |
| Logging | `LOGGING_ENABLED` |

---

## Git Workflow

Active development branch: `github-copilot`

Before every commit, run:

```powershell
ruff check .
ruff format --check .
python -m robotpy test
```

Before deploying to the robot:

```powershell
python write_version.py
git add version.txt
git commit -m "chore: update version.txt for deploy"
python -m robotpy deploy
```
