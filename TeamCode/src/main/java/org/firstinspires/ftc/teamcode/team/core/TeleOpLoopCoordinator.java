package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;

/**
 * Coordinates high-level TeleOp loop phases while preserving existing coordinator behavior.
 */
public class TeleOpLoopCoordinator {

    private static class PoseSnapshot {
        public final double x;
        public final double y;
        public final double headingRadians;

        public PoseSnapshot(double x, double y, double headingRadians) {
            this.x = x;
            this.y = y;
            this.headingRadians = headingRadians;
        }
    }

    /**
     * Mutable loop working state used only inside a single runLoopIteration call.
     * Keeping one object avoids juggling many small return classes between phases.
     */
    private static class LoopAccumulator {
        public boolean isAutoParking;
        public double autoParkStartTime;
        public String autoAlliance;
        public DarienOpModeFSM.ShootingPowerModes shootingPowerMode;
        public DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch;
        public PoseSnapshot pose;

        public LoopAccumulator(TeleOpLoopState loopState) {
            this.isAutoParking = loopState.isAutoParking;
            this.autoParkStartTime = loopState.autoParkStartTime;
            this.autoAlliance = loopState.autoAlliance;
            this.shootingPowerMode = loopState.shootingPowerMode;
            this.shotgunPowerLatch = loopState.shotgunPowerLatch;
        }
    }

    /**
     * Immutable per-iteration references shared by phase helper methods.
     */
    private static class PhaseContext {
        public final LoopAccumulator accumulator;
        public final DriverOneBindings driverOne;
        public final DriverTwoBindings driverTwo;
        public final TeleOpLoopMetrics metrics;
        public final TeleOpLoopDependencies deps;

        public PhaseContext(
                LoopAccumulator accumulator,
                DriverOneBindings driverOne,
                DriverTwoBindings driverTwo,
                TeleOpLoopMetrics metrics,
                TeleOpLoopDependencies deps
        ) {
            this.accumulator = accumulator;
            this.driverOne = driverOne;
            this.driverTwo = driverTwo;
            this.metrics = metrics;
            this.deps = deps;
        }
    }


    /**
     * Phase 1: always-run systems (drive, follower update, FSM updates, vision polling).
     */
    private void runAlwaysPhase(PhaseContext context) {
        // Update auto-park progress if active.
        updateAutoParkIfActive(context);

        // Apply driver-requested drive, then update follower position.
        applyDriveAndUpdateFollower(context);

        // Update mechanism FSMs (gate, turret, intake, shooting).
        updateMechanismFSMs(context);

        // Capture current robot pose for later use.
        capturePoseSnapshot(context);

        // Poll camera and update turret vision state.
        updateCameraControl(context);
    }

    private void updateAutoParkIfActive(PhaseContext context) {
        AutoParkCoordinator.AutoParkResult progressResult = context.deps.autoParkCoordinator.updateAutoParkProgress(
                context.accumulator.isAutoParking,
                context.accumulator.autoParkStartTime,
                context.metrics.currentTime,
                context.driverOne.driveStrafeAxis,
                context.driverOne.driveForwardAxis,
                context.driverOne.driveTurnAxis,
                context.deps.config.autoParkStickDeadzone,
                context.deps.config.autoParkTimeout,
                context.deps.follower,
                context.deps.telemetry,
                context.accumulator.shotgunPowerLatch
        );
        context.accumulator.isAutoParking = progressResult.isAutoParking;
        context.accumulator.autoParkStartTime = progressResult.autoParkStartTime;
        context.accumulator.shotgunPowerLatch = progressResult.shotgunPowerLatch;
    }

    private void applyDriveAndUpdateFollower(PhaseContext context) {
        context.deps.driveControlCoordinator.applyTeleOpDrive(
                context.accumulator.isAutoParking,
                context.driverOne.driveForwardAxis,
                context.driverOne.driveStrafeAxis,
                context.driverOne.driveTurnAxis,
                context.deps.config.driveDeadzone,
                context.deps.config.inputExponent,
                context.deps.config.speedScale,
                context.deps.config.speedScaleTurn,
                context.deps.config.rotationScale,
                context.deps.follower
        );
        context.deps.follower.update();
    }

    private void updateMechanismFSMs(PhaseContext context) {
        context.deps.gateFSM.update(context.metrics.currentTime, context.deps.telemetry);
        context.deps.turretFSM.update(context.metrics.currentTime, context.deps.telemetry);
        context.deps.intakeCoordinator.updateActiveIntake(context.metrics.currentTime, context.deps.telemetry);
        context.deps.shootingCoordinator.updateActiveSequence(context.metrics.currentTime, context.deps.telemetry);
    }

    private void capturePoseSnapshot(PhaseContext context) {
        context.accumulator.pose = new PoseSnapshot(
                context.deps.follower.getPose().getX(),
                context.deps.follower.getPose().getY(),
                context.deps.follower.getPose().getHeading()
        );
    }

    private void updateCameraControl(PhaseContext context) {
        context.deps.turretVisionCoordinator.updateCameraControl(
                context.metrics.currentTime,
                context.accumulator.autoAlliance,
                context.accumulator.pose.x,
                context.accumulator.pose.y,
                context.accumulator.pose.headingRadians,
                context.deps.telemetry
        );
    }

    /**
     * Phase 2: driver one commands (intake, auto-park start, odometry reset, shoot trigger).
     */
    private void runDriverOnePhase(PhaseContext context) {
        // Process driver one intake and eject commands.
        handleIntakeFromDriver(context);

        // Try to start auto-park if requested.
        tryStartAutoParkSequence(context);

        // Process driver two shooting controls.
        handleShootingFromDriver(context);

        // Reset odometry to human player corner if requested.
        tryResetOdometry(context);
    }

    private void handleIntakeFromDriver(PhaseContext context) {
        context.deps.intakeCoordinator.handleDriverControls(
                context.driverOne.intakeRequested,
                context.driverOne.ejectRequested,
                context.driverOne.intakeOffRequested
        );
    }

    private void tryStartAutoParkSequence(PhaseContext context) {
        AutoParkCoordinator.AutoParkResult startResult = context.deps.autoParkCoordinator.tryStartAutoPark(
                context.driverOne.autoParkRequested,
                context.accumulator.isAutoParking,
                context.metrics.currentTime,
                context.accumulator.autoParkStartTime,
                context.accumulator.autoAlliance,
                context.deps.config.parkRedX,
                context.deps.config.parkRedY,
                context.deps.config.parkRedHeadingDeg,
                context.deps.config.parkBlueX,
                context.deps.config.parkBlueY,
                context.deps.config.parkBlueHeadingDeg,
                context.deps.config.autoParkPower,
                context.deps.follower,
                context.deps.intakeFSM,
                context.deps.shotgunFSM,
                context.deps.shootingFSM,
                context.deps.turretFSM,
                context.deps.gateFSM,
                context.accumulator.shotgunPowerLatch
        );
        context.accumulator.isAutoParking = startResult.isAutoParking;
        context.accumulator.autoParkStartTime = startResult.autoParkStartTime;
        context.accumulator.shotgunPowerLatch = startResult.shotgunPowerLatch;
    }

    private void handleShootingFromDriver(PhaseContext context) {
        context.deps.shootingCoordinator.handleDriverControls(
                context.metrics.currentTime,
                context.driverTwo.closeGateRequested,
                context.driverTwo.shootPressed,
                context.driverTwo.shootReleased,
                context.driverTwo.shootingStickY,
                context.deps.config.shootPowerSelectStickThreshold
        );
    }

    private void tryResetOdometry(PhaseContext context) {
        context.deps.odometryResetCoordinator.tryResetToHumanPlayerPosition(
                context.driverOne.odometryResetRequested,
                context.accumulator.autoAlliance,
                context.deps.config.humanPlayerRedX,
                context.deps.config.humanPlayerRedY,
                context.deps.config.humanPlayerBlueX,
                context.deps.config.humanPlayerBlueY,
                context.deps.config.robotCenterOffsetX,
                context.deps.config.robotCenterOffsetY,
                context.deps.localizationService,
                context.deps.telemetry
        );
    }

    /**
     * Phase 3: driver two commands (alliance/turret mode, turret aim, shooter power selection).
     */
    private void runDriverTwoPhase(PhaseContext context) {
        // Handle driver requests to switch alliance and update internal state.
        handleAllianceSwitch(context);

        // Handle driver requests to switch turret aiming mode.
        handleTurretModeSwitch(context);

        // Apply manual or odometry-based turret control.
        applyTurretControl(context);

        // Compute and apply shooter power (close/far RPM) based on mode and position.
        handleShooterPowerSelection(context);
    }

    private void handleAllianceSwitch(PhaseContext context) {
        TurretVisionCoordinator.AllianceSwitchResult allianceSwitchResult = context.deps.turretVisionCoordinator.handleAllianceButtons(
                context.driverTwo.setAllianceRedRequested,
                context.driverTwo.setAllianceBlueRequested,
                context.accumulator.autoAlliance,
                context.deps.telemetry
        );
        context.accumulator.autoAlliance = allianceSwitchResult.alliance;
    }

    private void handleTurretModeSwitch(PhaseContext context) {
        TurretModeCoordinator.ModeSwitchResult modeSwitchResult = context.deps.turretModeCoordinator.handleModeSwitches(
                context.driverTwo.odometryModeRequested,
                context.driverTwo.cameraModeRequested,
                context.accumulator.shootingPowerMode
        );
        context.accumulator.shootingPowerMode = modeSwitchResult.shootingPowerMode;
        if (modeSwitchResult.shouldStartGoalReading) {
            context.deps.turretVisionCoordinator.startReadingGoalId(context.metrics.currentTime);
        }
    }

    private void applyTurretControl(PhaseContext context) {
        context.deps.turretCoordinator.applyManualOrOdometryControl(
                context.accumulator.autoAlliance,
                context.driverTwo.turretManualAxis,
                context.driverTwo.turretSpeedTrigger,
                context.driverTwo.turretCenterRequested,
                context.accumulator.pose.x,
                context.accumulator.pose.y,
                context.accumulator.pose.headingRadians
        );
    }

    private void handleShooterPowerSelection(PhaseContext context) {
        ShooterPowerCoordinator.PowerState powerState = context.deps.shooterPowerCoordinator.computePowerState(
                context.accumulator.shootingPowerMode,
                context.accumulator.shotgunPowerLatch,
                context.accumulator.pose.y,
                context.deps.config.shootingPowerOdometryYThreshold,
                context.driverTwo.shootingStickY,
                context.deps.config.shootPowerSelectStickThreshold,
                context.driverTwo.toggleShotgunPowerLatchRequested,
                context.driverTwo.forceShotgunLowPowerRequested
        );
        context.accumulator.shootingPowerMode = powerState.mode;
        context.accumulator.shotgunPowerLatch = powerState.latch;

        context.deps.shooterPowerCoordinator.applyRequestedPower(
                context.deps.shotgunFSM,
                context.accumulator.shotgunPowerLatch,
                context.deps.config.closeRpm,
                context.deps.config.farRpm,
                context.deps.telemetry
        );
    }

    private TeleOpStatusSnapshot buildStatusSnapshot(PhaseContext context) {
        return new TeleOpStatusSnapshot(
                context.deps.gateFSM,
                context.deps.intakeFSM,
                context.deps.shootingFSM,
                context.deps.turretFSM,
                context.accumulator.shootingPowerMode.toString(),
                context.accumulator.shotgunPowerLatch.toString(),
                context.metrics.ejectionMotorRpm,
                context.metrics.ejectionMotorPower,
                context.metrics.ejectionMotorVelocity,
                context.accumulator.autoAlliance,
                context.deps.turretVisionCoordinator.getTargetGoalTagId(),
                context.accumulator.pose.x,
                context.accumulator.pose.y,
                context.accumulator.pose.headingRadians,
                context.accumulator.isAutoParking,
                context.deps.config.parkRedX,
                context.deps.config.parkRedY,
                context.deps.config.parkBlueX,
                context.deps.config.parkBlueY,
                context.deps.config.autoParkTimeout,
                context.accumulator.autoParkStartTime,
                context.metrics.currentTime
        );
    }

    private void runAllPhases(PhaseContext context) {
        runAlwaysPhase(context);
        runDriverOnePhase(context);
        runDriverTwoPhase(context);
    }

    private TeleOpStatusCoordinator.TraceState publishTraceState(PhaseContext context) {
        TeleOpStatusSnapshot status = buildStatusSnapshot(context);
        return context.deps.statusCoordinator.publishStatus(
                context.deps.telemetry,
                context.deps.telemetryCoordinator,
                context.deps.turretVisionCoordinator,
                status
        );
    }

    private TeleOpLoopState buildNextState(PhaseContext context) {
        return new TeleOpLoopState(
                context.accumulator.isAutoParking,
                context.accumulator.autoParkStartTime,
                context.accumulator.autoAlliance,
                context.accumulator.shootingPowerMode,
                context.accumulator.shotgunPowerLatch
        );
    }

    public TeleOpIterationResult runLoopIteration(
            TeleOpLoopContext loopContext,
            TeleOpLoopIterationInput iterationInput
    ) {
        TeleOpLoopState loopState = loopContext.state;
        TeleOpLoopDependencies deps = loopContext.dependencies;
        LoopAccumulator accumulator = new LoopAccumulator(loopState);
        DriverOneBindings driverOne = iterationInput.driverOne;
        DriverTwoBindings driverTwo = iterationInput.driverTwo;
        TeleOpLoopMetrics metrics = iterationInput.metrics;

        PhaseContext context = new PhaseContext(accumulator, driverOne, driverTwo, metrics, deps);

        runAllPhases(context);
        TeleOpStatusCoordinator.TraceState traceState = publishTraceState(context);
        TeleOpLoopState nextState = buildNextState(context);
        return new TeleOpIterationResult(loopContext.withState(nextState), traceState);
    }
}
