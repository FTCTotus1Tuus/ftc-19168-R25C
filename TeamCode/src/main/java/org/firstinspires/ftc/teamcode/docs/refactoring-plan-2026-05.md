# FTC 19168 Refactoring Plan (May 2026)

## Scope and intent
- This is a **planning-only** document (no implementation yet).
- Goal: improve reliability, iteration speed, and maintainability for FTC SDK 11 + Pedro Pathing 2.0.4 + FSM stack.
- Focus areas: architecture boundaries, deterministic control loops, localization confidence, vision integration, and autonomous maintainability.

## Current architecture snapshot (from code review)
- Core base class: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/team/fsm/DarienOpModeFSM.java` (hardware init + shared constants + FSM wiring + follower).
- TeleOp orchestration: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/team/fsm/TeleOpFSM.java` (driver IO, subsystem updates, auto-park, alliance/pose restore).
- Autos: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/team/autosPedroPathing/RedGoalSide1.java` and `RedGoalSide2.java` with per-opmode path/state logic.
- Subsystems: FSM classes under `team/fsm` (`IntakeFSM`, `GateFSM`, `TurretFSM`, `ShootingFSM`, `ShotgunFSM`, `AprilTagDetectionFSM`, etc.).
- Pathing/localization config: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java` + tuner suite in `pedroPathing/Tuning.java`.
- Cross-opmode data handoff: `SharedPreferences` (`ftc_prefs`) for alliance and final odometry pose.

## Key architectural risks to address
- `DarienOpModeFSM` is a god-object (hardware map, constants, vision, subsystem factories, utility math, mode policy).
- TeleOp and Auto loops duplicate orchestration concerns (state progression, timeout handling, shooter lifecycle, pose persistence).
- Many control constants are scattered across classes; no single source of truth per subsystem/profile (practice vs match).
- Vision lifecycle is partially wired (`AprilTagProcessor` created, `VisionPortal` ownership unclear), which risks runtime inconsistency.
- Autonomous state machines are hand-coded per opmode; path/state reuse is low and regression risk is high when tuning.

## 2026 best-practice targets (gm0, FTC docs, FTCLib patterns, top-team conventions)
- Keep **subsystem logic independent of OpMode class** (OpMode = composition/orchestration only).
- Separate **robot hardware layer** (device mapping + safe defaults) from behavior layer.
- Use deterministic periodic update ordering (`input -> state update -> actuator output -> telemetry`) every loop.
- Prefer declarative autonomous building blocks (actions/commands/steps) over large switch-case monoliths.
- Treat localization/vision as services with health signals and explicit fallbacks (not implicit side effects).
- Keep tunables centralized and profile-based; preserve dashboard tunability while improving ownership.

## Refactoring roadmap (step-by-step)

### Phase 0 - Baseline and safety net
- Freeze behavior baselines: capture TeleOp and both Red autos expected sequence/timings.
- Add lightweight trace telemetry schema (state, timers, pose, turret mode, shooter mode) reused across opmodes.
- Document hardware assumptions and coordinate conventions in one place.

### Phase 1 - Layered robot container
- Introduce a `RobotContainer` (or similarly named class) that owns hardware, follower, and subsystem instances.
- Move hardware acquisition/default configuration out of `DarienOpModeFSM.initControls()` into dedicated classes:
  - `RobotHardware` (motors/servos/sensors mapping)
  - `RobotServices` (Follower, AprilTag service, preferences service)
- Keep `DarienOpModeFSM` as thin lifecycle shell that delegates to the container.

### Phase 2 - Subsystem contract cleanup
- Standardize subsystem interface shape: `init()`, `readSensors()`, `update()`, `writeOutputs()` where applicable.
- Remove direct cross-subsystem side effects where possible (e.g., shooting commanding intake internals) via explicit coordination layer.
- Keep FSM enums internal; expose intent methods only (`startIntaking()`, `requestShoot()`, etc.).

### Phase 3 - Vision and localization service hardening
- Create `AprilTagService` with explicit lifecycle (`start`, `poll`, `stop`) and immutable detection snapshots.
- Make camera control policy explicit (manual exposure/gain profile + fallback profile) with validation telemetry.
- Formalize odometry ownership: Pinpoint reset/seed operations happen in one place; runtime pose reads come from `follower.getPose()` only.
- Add confidence/fallback policy for turret aiming (`CAMERA -> ODOMETRY`) based on recency + quality criteria.

### Phase 4 - Autonomous framework extraction
- Extract common auto runner primitives from `RedGoalSide1`/`RedGoalSide2`:
  - reusable path library builder
  - reusable step types (follow path, spin shooter, shoot sequence, intake until condition, park)
  - centralized timeout strategy and fail-safe transitions
- Convert autos into compact plans built from shared primitives; keep opmode classes mostly declarative.

### Phase 5 - Configuration and tuning architecture
- Consolidate constants into scoped config classes (`DriveConfig`, `ShooterConfig`, `TurretConfig`, `VisionConfig`, `AutoConfig`).
- Keep `@Config` on tuned values, but remove unrelated constants from base opmode classes.
- Add named presets (practice field, competition field, battery profile) with explicit selection at init.
- Keep Pedro tuning workflow anchored to `pedroPathing/Tuning.java`; treat measured values as authoritative inputs to `Constants.java`.

### Phase 6 - Driver-control architecture improvements
- Isolate gamepad mapping into command-style bindings to reduce `TeleOpFSM` branching complexity.
- Separate drive control, shooter control, turret control, and mode toggles into focused coordinators.
- Preserve current behaviors (auto-park, odometry reset, alliance switching) while making logic testable and traceable.

## Proposed file/package direction (non-breaking migration)
- `team/core/` -> lifecycle + scheduler/update loop orchestration
- `team/hardware/` -> hardware map classes and device wrappers
- `team/subsystems/` -> FSM/mechanism logic (migrated from `team/fsm/` gradually)
- `team/services/` -> follower/localization, vision, preferences persistence
- `team/auto/` -> path libraries + step framework + opmode compositions

## Success criteria (before/after checks)
- TeleOp loop maintains stable cycle time while preserving existing controls and turret/shooter behavior.
- Red autos still complete intended cycles with equal or better timing and fewer timeout edge failures.
- Odometry handoff Auto->TeleOp remains intact and easier to reason about.
- AprilTag aiming behavior is deterministic, with explicit fallback telemetry.
- New autos can be authored by composing reusable steps rather than copy/pasting large switch statements.

## Implementation order for our upcoming walkthrough
1. Build `RobotHardware` + `RobotContainer` skeleton without behavior change.
2. Migrate `DarienOpModeFSM` init responsibilities into container.
3. Extract common auto step primitives from `RedGoalSide1`/`RedGoalSide2`.
4. Introduce vision/localization service boundaries and fallback policy.
5. Refactor `TeleOpFSM` input handling into command-style coordinators.
6. Consolidate config classes and clean up constants ownership.

