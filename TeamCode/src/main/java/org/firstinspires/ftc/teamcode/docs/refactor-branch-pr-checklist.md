# FTC 19168 Refactor Branch/PR Checklist (2026)

Use this checklist with `refactoring-plan-2026-05.md` to run the architecture refactor safely during season work.

## Branch model

- Keep `main` competition-safe at all times.
- Create umbrella branch: `feature/2026-architecture-refactor`.
- Create short-lived phase branches from umbrella:
  - `feature/refactor-p0-baseline`
  - `feature/refactor-p1-robot-container`
  - `feature/refactor-p2-subsystem-contracts`
  - `feature/refactor-p3-vision-localization-services`
  - `feature/refactor-p4-auto-framework`
  - `feature/refactor-p5-config-architecture`
  - `feature/refactor-p6-teleop-coordinators`
- Merge each phase branch into umbrella after verification.
- Open one final integration PR from umbrella to `main`.

## Required PR checklist (every phase)

- [ ] Scope is limited to one refactor phase.
- [ ] No unintended edits outside `TeamCode/`.
- [ ] Driver-facing OpMode names still exist (or migration note added).
- [ ] Build passes locally: `:TeamCode:assembleDebug`.
- [ ] TeleOp smoke test on robot passes:
  - [ ] Drive
  - [ ] Intake
  - [ ] Shooter
  - [ ] Turret mode switching
- [ ] Auto smoke test on robot passes:
  - [ ] `RedGoalSide1`
  - [ ] `RedGoalSide2`
- [ ] Auto -> TeleOp handoff via `ftc_prefs` is verified.
- [ ] PR notes include rollback plan and expected no-change behavior.

## Phase PR goals and exit criteria

### P0 - Baseline and safety net
**Branch:** `feature/refactor-p0-baseline`

Goals
- [ ] Add shared trace telemetry schema across TeleOp/autos (state, timers, pose, turret mode, shooter mode).
- [ ] Add/refresh architecture notes in docs.
- [ ] No behavior changes.

Exit criteria
- [ ] Telemetry appears in TeleOp and Red autos.
- [ ] Robot behavior matches pre-refactor baseline.

### P1 - Robot container skeleton
**Branch:** `feature/refactor-p1-robot-container`

Goals
- [ ] Introduce `RobotHardware` and `RobotContainer` skeletons.
- [ ] Move hardware mapping/default setup out of `DarienOpModeFSM` into container/hardware classes.
- [ ] Keep runtime behavior unchanged.

Exit criteria
- [ ] `DarienOpModeFSM` is thinner and delegates init responsibility.
- [ ] No startup regressions.

### P2 - Subsystem contract cleanup
**Branch:** `feature/refactor-p2-subsystem-contracts`

Goals
- [ ] Normalize subsystem interfaces and lifecycle usage.
- [ ] Reduce direct subsystem-to-subsystem side effects.
- [ ] Preserve FSM semantics and tunables.

Exit criteria
- [ ] TeleOp/autos compile and pass smoke tests.
- [ ] Shooting/intake/turret behavior remains equivalent.

### P3 - Vision and localization services
**Branch:** `feature/refactor-p3-vision-localization-services`

Goals
- [ ] Add explicit `AprilTagService` lifecycle (`start`, `poll`, `stop`).
- [ ] Use immutable detection snapshots.
- [ ] Centralize odometry seed/reset ownership.
- [ ] Implement explicit camera->odometry fallback policy with telemetry.

Exit criteria
- [ ] Stable AprilTag processing with no concurrency regressions.
- [ ] Deterministic turret fallback behavior.

### P4 - Autonomous framework extraction
**Branch:** `feature/refactor-p4-auto-framework`

Goals
- [ ] Extract reusable auto primitives (follow path, spin up, shoot, intake-until, park, timeout).
- [ ] Rebuild `RedGoalSide1` and `RedGoalSide2` from shared primitives.
- [ ] Centralize timeout/fail-safe logic.

Exit criteria
- [ ] Both Red autos complete expected sequence reliably.
- [ ] Autos are shorter and more declarative.

### P5 - Config/tuning architecture
**Branch:** `feature/refactor-p5-config-architecture`

Goals
- [ ] Consolidate constants into scoped config classes (`DriveConfig`, `ShooterConfig`, `TurretConfig`, `VisionConfig`, `AutoConfig`).
- [ ] Keep dashboard tuning (`@Config`) available.
- [ ] Keep `pedroPathing/Tuning.java` workflow as authoritative for drive tuning.

Exit criteria
- [ ] No duplicated/conflicting constants left in legacy locations.
- [ ] Tuning workflow remains functional.

### P6 - TeleOp coordinator split
**Branch:** `feature/refactor-p6-teleop-coordinators`

Goals
- [ ] Extract gamepad mapping into focused coordinators/bindings.
- [ ] Reduce branching complexity in `TeleOpFSM`.
- [ ] Preserve current control behavior (auto-park, odometry reset, alliance switch, turret/shooter controls).

Exit criteria
- [ ] Driver experience is unchanged.
- [ ] TeleOp loop remains stable under sustained input.

## Final integration PR to `main`

- [ ] All phase branches merged into umbrella branch.
- [ ] Full test matrix summary included in PR description.
- [ ] Driver/operator impact summary included (what changed, what did not).
- [ ] Deployment tag/commit selected for next field test or event.

## Suggested command snippets

```powershell
git checkout main
git pull

git checkout -b feature/2026-architecture-refactor

git checkout -b feature/refactor-p0-baseline
# ...work, commit, PR to umbrella...

git checkout feature/2026-architecture-refactor
git checkout -b feature/refactor-p1-robot-container
# ...repeat for each phase...
```

