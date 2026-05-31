# Robot Skeleton Architecture Guide

This document explains how to build and extend the FTC robot codebase so it stays clean, testable, and easy to grow during the season.
It is written for both junior developers and AI coding agents.

The goal is to start with a **barebones but complete robot skeleton**:
- the robot boots reliably
- hardware is mapped in one place
- OpModes stay thin
- subsystems keep their own behavior
- configuration values live in dedicated config classes
- TeleOp and Autonomous share the same base structure

---

## 1. Core design principles

Follow these rules when adding or changing code:

1. **OpModes should orchestrate, not own everything.**
   - `TeleOpFSM` and autonomous OpModes should coordinate behavior.
   - They should not contain all hardware mapping, camera setup, or subsystem wiring.

2. **Separate hardware, services, and behavior.**
   - `team.hardware` = device mapping and safe startup defaults.
   - `team.services` = follower, vision, localization, and preferences.
   - `team.fsm` = mechanism logic and OpMode state machines.
   - `team.config` = all tunable constants.

3. **Use intent methods, not direct state mutation.**
   - Good: `intakeFSM.startIntaking()`
   - Avoid: setting FSM enum values directly from outside the class.

4. **Keep one source of truth for constants.**
   - Drive values belong in `DriveConfig`.
   - Shooter values belong in `ShooterConfig`.
   - Turret geometry belongs in `TurretConfig`.
   - Vision values belong in `VisionConfig`.
   - Autonomous path and parking values belong in `AutoConfig`.

5. **Make the loop deterministic.**
   - Read driver input.
   - Update subsystem state.
   - Write actuator outputs.
   - Publish telemetry.

6. **Favor composition over a giant base class.**
   - `DarienOpModeFSM` should be a thin lifecycle shell.
   - Real work should happen in `RobotContainer`, services, and subsystem classes.

---

## 2. Package layout

Use this package structure as the default organization for team code:

- `team/fsm/`
  - OpMode base class
  - subsystem FSMs
  - TeleOp and autonomous logic
- `team/core/`
  - robot composition and lifecycle coordination
- `team/hardware/`
  - device mapping and hardware startup defaults
- `team/services/`
  - follower, localization, vision, preferences, and other shared services
- `team/config/`
  - tuning constants exposed to FTC Dashboard
- `team/autosPedroPathing/`
  - autonomous OpModes and path plans
- `team/testing/`
  - debug and tuning OpModes only

If you need to add a new file, place it in the package that matches its responsibility.
Do not put unrelated logic into `DarienOpModeFSM` just because it is convenient.

---

## 3. Java class architecture and inheritance structure

The most important inheritance rule is simple:

- `LinearOpMode` is the FTC SDK entry point.
- `DarienOpModeFSM` extends `LinearOpMode`.
- Every TeleOp and autonomous OpMode extends `DarienOpModeFSM`.

### 3.1 Inheritance tree

```text
LinearOpMode
└── DarienOpModeFSM
    ├── TeleOpFSM
    ├── RedGoalSide1
    ├── RedGoalSide2
    ├── BlueGoalSide1
    ├── BlueAudience1
    └── other OpModes
```

### 3.2 Composition tree

`DarienOpModeFSM` should not directly own all robot logic forever.
Instead, it should hold a `RobotContainer`, which composes the rest of the robot:

```text
DarienOpModeFSM
└── RobotContainer
    ├── RobotHardware
    ├── RobotServices
    │   ├── AprilTagVisionService
    │   ├── AprilTagService
    │   ├── LocalizationService
    │   └── PreferencesService
    ├── Drive/Follower resources
    ├── Config classes
    └── subsystem FSM instances
        ├── GateFSM
        ├── IntakeFSM
        ├── TurretFSM
        ├── ShootingFSM
        ├── ShotgunFSM
        ├── ShootArtifactFSM
        └── ShootPatternFSM
```

### 3.3 What each class should do

#### `DarienOpModeFSM`
- common OpMode base class
- shared telemetry setup
- decides whether the OpMode is autonomous or TeleOp
- creates or owns the `RobotContainer`
- exposes helper utilities that are truly generic
- should stay thin and stable

#### `RobotContainer`
- central composition root for a robot instance
- creates hardware, services, and subsystem FSMs in one place
- returns references to the robot pieces that the OpMode needs
- handles lifecycle cleanup such as camera shutdown

#### `RobotHardware`
- maps hardware devices using `hardwareMap`
- applies safe startup defaults
- keeps raw device names and device setup in one place
- does **not** contain robot behavior

#### `RobotServices`
- owns non-hardware services that are shared by multiple subsystems
- creates and manages the follower, vision service, localization service, and preferences service
- handles service lifecycle and teardown

#### `RobotHardware` vs `RobotServices`
- if it is a physical device, it belongs in hardware
- if it is a software wrapper around device behavior or shared state, it belongs in services

#### Subsystem FSM classes in `team/fsm`
- each class controls one mechanism or one behavior area
- each class should have a small public API
- each FSM should hide its internal enum state
- outside code should call intent methods, not edit fields directly

---

## 4. Skeleton robot startup flow

When an OpMode starts, the startup flow should be predictable.

### 4.1 Typical init sequence

1. FTC SDK constructs the OpMode.
2. `runOpMode()` begins.
3. `initControls()` is called.
4. `DarienOpModeFSM` creates or reuses `RobotContainer`.
5. `RobotContainer.initialize()` runs:
   - `RobotHardware.initialize(hardwareMap)`
   - `RobotServices.initialize()`
   - subsystem FSM construction and `init()` calls
6. `follower`, camera, and preferences services become available.
7. Telemetry confirms the robot is ready.

### 4.2 What should be ready after init

A skeleton robot should reliably initialize these items even before the season’s full mechanisms are finished:

- drivetrain follower
- hardware map for drive and core mechanisms
- one shooter/flywheel device placeholder
- turret and intake subsystems if installed
- shared preferences for auto-to-teleop handoff
- vision service, even if it falls back safely when the camera is unavailable

### 4.3 What should happen on stop

When the OpMode ends:
- close the vision portal
- stop camera resources cleanly
- leave shared state in a known state

---

## 5. Barebones season-start robot requirements

At the start of the season, build only the minimum robot skeleton needed to support future work.

### 5.1 Required systems

- **Drive base**
  - map drivetrain motors
  - create the Pedro Pathing follower
  - verify pose reads from `follower.getPose()`

- **Hardware layer**
  - map every physical device once
  - set zero-power behavior and direction defaults in one place

- **Preferences handoff**
  - save alliance and final pose after autonomous
  - restore pose at TeleOp init when available

- **Telemetry**
  - show state, pose, turret mode, shooter mode, and major health signals

### 5.2 Optional systems that should fail safely

If the camera is not ready, the robot should still run with odometry-only fallback.
If a mechanism is unfinished, keep the class present but make its behavior safe and minimal.

Do **not** block robot startup just because an optional system is not ready.

---

## 6. How to add a new subsystem correctly

When adding a mechanism or behavior, use this pattern:

1. Create or update a config class if tuning is needed.
2. Add hardware mapping only if the mechanism needs a new device.
3. Create a subsystem FSM in `team/fsm`.
4. Expose only intent methods from that subsystem.
5. Wire the subsystem through `RobotContainer`.
6. Call it from `TeleOpFSM` or autonomous only through the container reference.

### Example

If you add a new arm:
- `team/config/ArmConfig.java`
- `team/hardware/RobotHardware.java` for the motor/servo mapping
- `team/fsm/ArmFSM.java` for the behavior
- `team/core/RobotContainer.java` to compose it
- `TeleOpFSM` uses `armFSM.extend()` or `armFSM.retract()`

---

## 7. What OpModes should look like

### 7.1 TeleOp

`TeleOpFSM` should:
- read gamepad input
- select control modes
- call subsystem update methods
- manage driver-facing decisions such as turret mode switching and auto-park
- keep branching understandable

TeleOp should not contain low-level hardware mapping.

### 7.2 Autonomous

Autonomous OpModes should:
- extend `DarienOpModeFSM`
- build paths and execute a small state machine
- reuse shared helpers where possible
- write alliance and final pose to preferences

Autonomous should not repeat the same device wiring that already exists in `RobotContainer`.

---

## 8. Configuration ownership rules

Use config classes for all values that may change during tuning.

### Put values here
- `DriveConfig`
  - drive scaling
  - odometry reset offsets
  - encoder ticks per rotation
- `ShooterConfig`
  - flywheel power and RPM tuning
  - shooter thresholds
- `TurretConfig`
  - turret geometry and servo positions
- `VisionConfig`
  - AprilTag IDs
  - camera exposure and gain
  - vision timeouts
- `AutoConfig`
  - parking poses
  - auto timing
  - human-player reset poses

### Do not put these values here
- one-off hardware mapping strings
- control flow
- subsystem state
- behavior logic

Use `@Config` and `@Configurable` for values that should be adjustable from FTC Dashboard.

---

## 9. Localization and vision rules

The robot uses the follower as the runtime pose source.

### Localization rules
- seed pose in one place
- read runtime pose from `follower.getPose()`
- do not treat the hardware odometry device as the primary runtime pose API

### Vision rules
- the vision portal should be owned by a service
- camera startup and shutdown must be explicit
- AprilTag snapshots should be immutable or treated as read-only
- if vision is unhealthy, the robot should fall back to odometry or manual control

### Turret aiming rules
- `MANUAL` is driver-controlled
- `CAMERA` is AprilTag-based
- `ODOMETRY` is pose-based
- fall back from camera to odometry when the camera is stale or unavailable

---

## 10. Good file-writing habits for AI agents

When an AI agent writes code in this repository, it should follow these rules:

- do not invent a second architecture root
- do not duplicate hardware mapping across multiple classes
- do not move config values into OpModes
- do not bypass `RobotContainer` just to save time
- preserve current public method names unless a refactor explicitly requires a change
- keep changes small and reviewable
- prefer adding a new class over adding another large switch statement

If unsure where code belongs, ask:

1. Is it hardware?
2. Is it shared service logic?
3. Is it subsystem behavior?
4. Is it a constant?
5. Is it only orchestration?

That question usually tells you the correct package.

---

## 11. Recommended skeleton robot implementation order

For a new season or a clean robot bring-up, build in this order:

1. **Hardware mapping only**
   - make the robot initialize safely
   - verify motors, servos, sensors, and the follower

2. **RobotContainer and services**
   - compose the robot in one place
   - confirm startup and shutdown work

3. **Drive and pose handling**
   - confirm odometry reset and runtime pose reads

4. **Minimal TeleOp loop**
   - drive controls first
   - then intake/shooter/turret controls

5. **Minimal autonomous**
   - one path
   - one scoring action
   - one parking action

6. **Expand by subsystem**
   - add features one mechanism at a time

This approach keeps the robot usable while the season code is still growing.

---

## 12. Final rule of thumb

If a class becomes hard to explain in one sentence, it probably owns too much.
Move the code into the correct layer instead of making `DarienOpModeFSM` larger.

The ideal end state is:
- OpModes are short
- hardware is centralized
- services are reusable
- subsystems are isolated
- config values are easy to tune
- the robot can be booted as a skeleton at the start of the season and extended safely later

---

## 13. Class-by-class file template

Use this template when creating a new class. Adapt the sections to the class role, but keep the same general structure.

### 13.1 Generic Java class template

```java
package org.firstinspires.ftc.teamcode.team.example;

// imports here

/**
 * Short description of what this class owns and why it exists.
 */
public class ExampleClass {

    // Fields

    public ExampleClass() {
        // initialize required dependencies only
    }

    public void init() {
        // safe startup work
    }

    public void update() {
        // periodic work
    }

    public void stop() {
        // release resources if needed
    }
}
```

### 13.2 OpMode template

```java
@TeleOp(name = "Example TeleOp")
public class ExampleTeleOp extends DarienOpModeFSM {

    @Override
    public void runOpMode() throws InterruptedException {
        initControls();

        waitForStart();

        while (opModeIsActive()) {
            // 1. read input
            // 2. update subsystem state
            // 3. write outputs
            // 4. send telemetry
            idle();
        }

        stopRobot();
    }
}
```

### 13.3 Subsystem FSM template

```java
public class ExampleFSM {

    private enum State {
        IDLE,
        ACTIVE
    }

    private State state = State.IDLE;

    public void init() {
        // map any stateful startup behavior here
    }

    public void start() {
        state = State.ACTIVE;
    }

    public void stop() {
        state = State.IDLE;
    }

    public void update(double currentTimeSec) {
        switch (state) {
            case IDLE:
                break;
            case ACTIVE:
                break;
        }
    }
}
```

### 13.4 What to include in every class header comment

Write one or two short sentences that answer these questions:
- What does this class own?
- What should other classes call here?
- What should other classes **not** do here?

---

## 14. First-week skeleton robot checklist

Use this checklist during the first days of a season to build a minimal but reliable robot base before adding advanced features.

### 14.1 Build order

- [ ] Create or confirm `RobotHardware` maps all installed devices.
- [ ] Create or confirm `RobotServices` owns follower, vision, localization, and preferences.
- [ ] Make `RobotContainer` compose hardware, services, and subsystem FSMs.
- [ ] Keep `DarienOpModeFSM` as a thin base class.
- [ ] Keep `TeleOpFSM` limited to driver input and orchestration.
- [ ] Keep autonomous OpModes limited to paths, state transitions, and pose handoff.

### 14.2 Minimum runtime checks

- [ ] Robot initializes without throwing during `initControls()`.
- [ ] Motors and servos are mapped exactly once.
- [ ] Follower is created and returns a valid pose.
- [ ] `follower.getPose()` is used for runtime pose reads.
- [ ] Auto-to-TeleOp pose handoff uses `PreferencesService`.
- [ ] Camera failure does not prevent the robot from driving.

### 14.3 Minimum TeleOp checks

- [ ] Driver can move the robot.
- [ ] Intake behavior can be toggled safely.
- [ ] Shooter can be disabled without breaking the loop.
- [ ] Turret control does not block driving.
- [ ] Telemetry shows pose and mechanism state.

### 14.4 Minimum autonomous checks

- [ ] OpMode starts and builds its paths.
- [ ] Robot follows at least one simple path.
- [ ] Robot can save its final pose.
- [ ] Robot can park or stop safely even if a scoring step fails.

### 14.5 Stop conditions

If any optional feature becomes unstable, pause that feature and keep the skeleton robot working.
Do not let vision, tuning, or a new mechanism break the core drive + pose + init flow.

