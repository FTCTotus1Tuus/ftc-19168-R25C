# Trace Telemetry Schema (P0 Baseline)

This document defines the shared trace keys emitted by `TeleOpFSM`, `RedGoalSide1`, and `RedGoalSide2` during the refactor baseline phase.

## Keys

- `TRACE/Phase` - high-level loop context (for example `TeleOp`, `Auto-RedGoalSide1`).
- `TRACE/State` - current state identifier (`DRIVER_CONTROL`, `AUTO_PARK`, or auto `pathState`).
- `TRACE/RuntimeSec` - OpMode runtime in seconds.
- `TRACE/StateTimerSec` - elapsed seconds in the current state context.
- `TRACE/Pose` - live follower pose as `x`, `y`, and heading in degrees.
- `TRACE/TurretMode` - current turret FSM mode.
- `TRACE/ShooterMode` - current shooting power mode (`MANUAL` or `ODOMETRY`).
- `TRACE/ShooterStage` - current shooting FSM stage.

## Notes

- Runtime pose values are sourced from `follower.getPose()`.
- This schema is intended for no-behavior-change validation while refactoring internals.
- Additional keys can be appended in later phases, but existing keys should remain stable for log comparisons.

