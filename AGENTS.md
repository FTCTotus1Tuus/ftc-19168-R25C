# AGENTS.md — FTC Team 19168 (DECODE 2025-2026)

## Project Overview
FTC robotics project built on the FTC SDK v11.0. Two Gradle modules: `FtcRobotController` (upstream SDK — do not edit) and `TeamCode` (all team code lives here). Android app targeting SDK 28, min SDK 24.

## Architecture
All team code is under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

- **`team/fsm/DarienOpModeFSM.java`** — Abstract base class extending `LinearOpMode`. Every OpMode (teleop and autonomous) extends this. It owns hardware initialization (`initControls()`), all FSM instances, tuning constants, and the Pedro Pathing `Follower`.
- **`team/fsm/TeleOpFSM.java`** — The main driver-controlled OpMode. Extends `DarienOpModeFSM`.
- **`team/autosPedroPathing/`** — Autonomous OpModes. Named `{Color}{GoalSide|Audience}{Number}` (e.g. `BlueGoalSide1`, `RedAudience2`). Each extends `DarienOpModeFSM`, builds Pedro Pathing `PathChain`s, and runs a state machine in a while-loop.
- **FSM subsystem classes** (`team/fsm/`): `GateFSM`, `IntakeFSM`, `ShotgunFSM`, `TurretFSM`, `ShootingFSM`, `ShootArtifactFSM`, `ShootPatternFSM`, `AprilTagDetectionFSM`. Each manages one robot mechanism with enum states, `update()` methods, and hardware control.
- **`pedroPathing/Constants.java`** — Pedro Pathing configuration: registers the **goBILDA Pinpoint** I2C odometry computer (hardware map name `"odo"`, `goBILDA_4_BAR_POD` encoder resolution, pod offsets in inches) as the localizer via `PinpointConstants`, along with drivetrain motor names (`omniMotor0`–`omniMotor3`), follower mass, and velocity tuning. `Constants.createFollower(hardwareMap)` is the single factory method — called once inside `DarienOpModeFSM.initControls()`.
- **`pedroPathing/Tuning.java`** — Unified Pedro Pathing tuning OpMode (`@TeleOp`, group "Pedro Pathing") built on `SelectableOpMode` (from `com.pedropathing:telemetry:1.0.0`). Provides an interactive menu of sub-tuners for localization, velocity, PIDF, and path tests. Use this for all drivetrain tuning instead of individual OpModes.
- **`team/MotorHelper.java`** — Custom PI/PID controller for motor velocity (used by `ShotgunFSM` for flywheel RPM control).
- **`team/testing/`** — Debug and tuning OpModes: `cameraDebugTest`, `ConceptAprilTagEasyDarien`, `ImageProcessDebug`, `TuneAprilTagExposure`. Not used in competition.

## Key Patterns

### FSM Convention
Each subsystem FSM follows the same pattern: an enum for states, a constructor taking `HardwareMap` or parent OpMode, an `init()` method, and an `update(currentTime, debug, telemetry)` method. State transitions are done via explicit methods (e.g. `gateFSM.open()`, `intakeFSM.startIntaking()`), never by setting the enum directly from outside.

### FTC Dashboard Tuning
Classes annotated with `@Config` and `@Configurable` expose `public static` fields to FTC Dashboard for live tuning. Use this for any new tuning constant — declare it as `public static double` (not `final`). Example: `DarienOpModeFSM.SHOT_GUN_POWER_UP_RPM`.

### Auto → TeleOp State Transfer
Alliance color and final odometry pose are passed from autonomous to teleop via Android `SharedPreferences` (key `"ftc_prefs"`). Auto writes `auto_alliance`, `auto_final_x/y/heading`; TeleOp reads them on init. This is critical for continuous odometry and correct turret targeting.

### Turret Aiming Modes
`TurretFSM` supports three modes: `MANUAL` (gamepad stick), `CAMERA` (AprilTag bearing via `AprilTagDetectionFSM`), and `ODOMETRY` (calculated from follower pose + goal coordinates). Camera and odometry modes use field constants `GOAL_RED_X/Y` and `GOAL_BLUE_X/Y`.

### Odometry (goBILDA Pinpoint)
The robot's sole position source is a **goBILDA Pinpoint** I2C odometry computer (hardware map name `"odo"`, class `com.qualcomm.hardware.gobilda.GoBildaPinpointDriver`). Pedro Pathing's `Follower` wraps it internally — **always read live pose via `follower.getPose()`**, not directly from the device. `TeleOpFSM` holds a direct `GoBildaPinpointDriver odo` reference only for pose resets at init: it calls `odo.setPosition(...)` to seed the Pinpoint with either the saved auto final pose (from `SharedPreferences`) or the default human-player-corner position, then syncs the follower with `follower.setPose(...)`. Turret `ODOMETRY` aiming and flywheel power selection both consume `follower.getPose()` at runtime.

### Field Coordinate System
Pedro Pathing coordinates: **(0, 0) = left audience corner (red loading zone)**, **(144, 144) = red goal corner**. Blue goal is at (0, 144). All path poses, park positions, and goal constants use this system.

### Flywheel Power Mode Selection
`DarienOpModeFSM.ShootingPowerModes` has two values: `MANUAL` and `ODOMETRY`. `initControls()` auto-sets `ODOMETRY` for `@Autonomous` OpModes and `MANUAL` for TeleOp (via `isAutonomousMode()`). In `ODOMETRY` mode, `ShootingFSM` selects FAR vs CLOSE RPM automatically based on whether robot Y ≤ `SHOOTING_POWER_ODOMETRY_Y_THRESHOLD` (48.0 in). In `TeleOpFSM`, `gamepad2.dpadUpWasPressed()` switches to `ODOMETRY`; any `gamepad2.right_stick_y` input reverts to `MANUAL`.

## Code Analysis Rules

### Verify Before Assuming
Before describing what any function argument, parameter, or method does, **look up the actual source code or library implementation** using available search tools. Do not infer behavior from argument names, variable names, or general knowledge about similar libraries — FTC/Pedro Pathing APIs are version-specific and often differ from documentation or intuition. For example:
- A boolean argument named or guessed to be `useHeadingCorrection` may actually control `useBrakeMode`, or vice versa.
- Always check the Pedro Pathing `Follower` source (or cached Gradle artifacts) before making claims about `setTeleOpDrive()`, `startTeleopDrive()`, `followPath()`, or any other API method arguments.
- If the source cannot be located, explicitly state the uncertainty rather than presenting a guess as fact.

## Build & Deploy
- Build: `./gradlew :TeamCode:assembleDebug` (or use Android Studio)
- Deploy: Connect to Robot Controller phone/Control Hub via USB or WiFi Direct, then Run from Android Studio
- Only edit files under `TeamCode/`; never modify `FtcRobotController/`, `build.common.gradle`, or `build.dependencies.gradle` unless adding a new library

## Dependencies (in `build.dependencies.gradle`)
| Library | Purpose |
|---|---|
| `com.pedropathing:ftc:2.0.4` | Autonomous path following (Bezier curves, PID heading control) |
| `com.pedropathing:telemetry:1.0.0` | Pedro Pathing telemetry integration |
| `com.acmerobotics.dashboard:dashboard:0.4.16` | Live tuning & telemetry via web dashboard |
| `org.openftc:easyopencv:1.7.3` | OpenCV vision pipeline (used in `ImageProcess.java`) |
| `com.bylazar:fullpanels:1.0.12` | Enhanced telemetry panels (`TelemetryManager`, `@Configurable`) |

## Hardware Map Names
Motor names: `omniMotor0`–`omniMotor3` (mecanum), `ejectionMotor` (flywheel), `rubberBandsFront` (intake). Servo names: `gateServo`, `turretServo`. CRServos: `rampServoLow`, `rampServoHigh`, `rubberBandsMid`, `intakeRear`. Sensors: `"odo"` (goBILDA Pinpoint odometry computer, `GoBildaPinpointDriver`), `"Webcam 1"`, `intakeColorSensor`, `middleColorSensor`, `turretColorSensor` (NormalizedColorSensor). LEDs: `LEDRight1`, `LEDLeft1`, `LEDRight2`, `LEDLeft2` (DigitalChannel). All names are string literals in the Java source — search for `hardwareMap.get(` to find them.

