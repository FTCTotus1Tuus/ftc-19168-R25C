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
     * Phase 1: always-run systems (drive, follower update, FSM updates, vision polling).
     */
    private void runAlwaysPhase(
            LoopAccumulator accumulator,
            DriverOneBindings driverOne,
            double currentTime,
            TeleOpLoopDependencies deps
    ) {
        AutoParkCoordinator.AutoParkResult progressResult = deps.autoParkCoordinator.updateAutoParkProgress(
                accumulator.isAutoParking,
                accumulator.autoParkStartTime,
                currentTime,
                driverOne.driveStrafeAxis,
                driverOne.driveForwardAxis,
                driverOne.driveTurnAxis,
                deps.config.autoParkStickDeadzone,
                deps.config.autoParkTimeout,
                deps.follower,
                deps.telemetry,
                accumulator.shotgunPowerLatch
        );
        accumulator.isAutoParking = progressResult.isAutoParking;
        accumulator.autoParkStartTime = progressResult.autoParkStartTime;
        accumulator.shotgunPowerLatch = progressResult.shotgunPowerLatch;

        deps.driveControlCoordinator.applyTeleOpDrive(
                accumulator.isAutoParking,
                driverOne.driveForwardAxis,
                driverOne.driveStrafeAxis,
                driverOne.driveTurnAxis,
                deps.config.driveDeadzone,
                deps.config.inputExponent,
                deps.config.speedScale,
                deps.config.speedScaleTurn,
                deps.config.rotationScale,
                deps.follower
        );

        deps.follower.update();

        deps.gateFSM.update(currentTime, deps.telemetry);
        deps.turretFSM.update(currentTime, deps.telemetry);

        deps.intakeCoordinator.updateActiveIntake(currentTime, deps.telemetry);
        deps.shootingCoordinator.updateActiveSequence(currentTime, deps.telemetry);

        accumulator.pose = new PoseSnapshot(
                deps.follower.getPose().getX(),
                deps.follower.getPose().getY(),
                deps.follower.getPose().getHeading()
        );

        deps.turretVisionCoordinator.updateCameraControl(
                currentTime,
                accumulator.autoAlliance,
                accumulator.pose.x,
                accumulator.pose.y,
                accumulator.pose.headingRadians,
                deps.telemetry
        );
    }

    /**
     * Phase 2: driver one commands (intake, auto-park start, odometry reset, shoot trigger).
     */
    private void runDriverOnePhase(
            LoopAccumulator accumulator,
            DriverOneBindings driverOne,
            DriverTwoBindings driverTwo,
            double currentTime,
            TeleOpLoopDependencies deps
    ) {
        deps.intakeCoordinator.handleDriverControls(
                driverOne.intakeRequested,
                driverOne.ejectRequested,
                driverOne.intakeOffRequested
        );

        AutoParkCoordinator.AutoParkResult startResult = deps.autoParkCoordinator.tryStartAutoPark(
                driverOne.autoParkRequested,
                accumulator.isAutoParking,
                currentTime,
                accumulator.autoParkStartTime,
                accumulator.autoAlliance,
                deps.config.parkRedX,
                deps.config.parkRedY,
                deps.config.parkRedHeadingDeg,
                deps.config.parkBlueX,
                deps.config.parkBlueY,
                deps.config.parkBlueHeadingDeg,
                deps.config.autoParkPower,
                deps.follower,
                deps.intakeFSM,
                deps.shotgunFSM,
                deps.shootingFSM,
                deps.turretFSM,
                deps.gateFSM,
                accumulator.shotgunPowerLatch
        );
        accumulator.isAutoParking = startResult.isAutoParking;
        accumulator.autoParkStartTime = startResult.autoParkStartTime;
        accumulator.shotgunPowerLatch = startResult.shotgunPowerLatch;

        deps.shootingCoordinator.handleDriverControls(
                currentTime,
                driverTwo.closeGateRequested,
                driverTwo.shootPressed,
                driverTwo.shootReleased,
                driverTwo.shootingStickY,
                deps.config.shootPowerSelectStickThreshold
        );

        deps.odometryResetCoordinator.tryResetToHumanPlayerPosition(
                driverOne.odometryResetRequested,
                accumulator.autoAlliance,
                deps.config.humanPlayerRedX,
                deps.config.humanPlayerRedY,
                deps.config.humanPlayerBlueX,
                deps.config.humanPlayerBlueY,
                deps.config.robotCenterOffsetX,
                deps.config.robotCenterOffsetY,
                deps.localizationService,
                deps.telemetry
        );
    }

    /**
     * Phase 3: driver two commands (alliance/turret mode, turret aim, shooter power selection).
     */
    private void runDriverTwoPhase(
            LoopAccumulator accumulator,
            DriverTwoBindings driverTwo,
            double currentTime,
            TeleOpLoopDependencies deps
    ) {
        TurretVisionCoordinator.AllianceSwitchResult allianceSwitchResult = deps.turretVisionCoordinator.handleAllianceButtons(
                driverTwo.setAllianceRedRequested,
                driverTwo.setAllianceBlueRequested,
                accumulator.autoAlliance,
                deps.telemetry
        );
        accumulator.autoAlliance = allianceSwitchResult.alliance;

        TurretModeCoordinator.ModeSwitchResult modeSwitchResult = deps.turretModeCoordinator.handleModeSwitches(
                driverTwo.odometryModeRequested,
                driverTwo.cameraModeRequested,
                accumulator.shootingPowerMode
        );
        accumulator.shootingPowerMode = modeSwitchResult.shootingPowerMode;
        if (modeSwitchResult.shouldStartGoalReading) {
            deps.turretVisionCoordinator.startReadingGoalId(currentTime);
        }

        deps.turretCoordinator.applyManualOrOdometryControl(
                accumulator.autoAlliance,
                driverTwo.turretManualAxis,
                driverTwo.turretSpeedTrigger,
                driverTwo.turretCenterRequested,
                accumulator.pose.x,
                accumulator.pose.y,
                accumulator.pose.headingRadians
        );

        ShooterPowerCoordinator.PowerState powerState = deps.shooterPowerCoordinator.computePowerState(
                accumulator.shootingPowerMode,
                accumulator.shotgunPowerLatch,
                accumulator.pose.y,
                deps.config.shootingPowerOdometryYThreshold,
                driverTwo.shootingStickY,
                deps.config.shootPowerSelectStickThreshold,
                driverTwo.toggleShotgunPowerLatchRequested,
                driverTwo.forceShotgunLowPowerRequested
        );
        accumulator.shootingPowerMode = powerState.mode;
        accumulator.shotgunPowerLatch = powerState.latch;

        deps.shooterPowerCoordinator.applyRequestedPower(
                deps.shotgunFSM,
                accumulator.shotgunPowerLatch,
                deps.config.closeRpm,
                deps.config.farRpm,
                deps.telemetry
        );
    }

    private TeleOpStatusSnapshot buildStatusSnapshot(
            LoopAccumulator accumulator,
            TeleOpLoopDependencies deps,
            TeleOpLoopMetrics metrics
    ) {
        return new TeleOpStatusSnapshot(
                deps.gateFSM,
                deps.intakeFSM,
                deps.shootingFSM,
                deps.turretFSM,
                accumulator.shootingPowerMode.toString(),
                accumulator.shotgunPowerLatch.toString(),
                metrics.ejectionMotorRpm,
                metrics.ejectionMotorPower,
                metrics.ejectionMotorVelocity,
                accumulator.autoAlliance,
                deps.turretVisionCoordinator.getTargetGoalTagId(),
                accumulator.pose.x,
                accumulator.pose.y,
                accumulator.pose.headingRadians,
                accumulator.isAutoParking,
                deps.config.parkRedX,
                deps.config.parkRedY,
                deps.config.parkBlueX,
                deps.config.parkBlueY,
                deps.config.autoParkTimeout,
                accumulator.autoParkStartTime,
                metrics.currentTime
        );
    }

    public TeleOpIterationResult runLoopIteration(
            TeleOpLoopContext loopContext,
            DriverOneBindings driverOne,
            DriverTwoBindings driverTwo,
            TeleOpLoopMetrics metrics
    ) {
        TeleOpLoopState loopState = loopContext.state;
        TeleOpLoopDependencies deps = loopContext.dependencies;
        LoopAccumulator accumulator = new LoopAccumulator(loopState);

        runAlwaysPhase(
                accumulator,
                driverOne,
                metrics.currentTime,
                deps
        );

        runDriverOnePhase(
                accumulator,
                driverOne,
                driverTwo,
                metrics.currentTime,
                deps
        );

        runDriverTwoPhase(
                accumulator,
                driverTwo,
                metrics.currentTime,
                deps
        );

        TeleOpStatusSnapshot status = buildStatusSnapshot(
                accumulator,
                deps,
                metrics
        );

        TeleOpStatusCoordinator.TraceState traceState = deps.statusCoordinator.publishStatus(
                deps.telemetry,
                deps.telemetryCoordinator,
                deps.turretVisionCoordinator,
                status
        );

        TeleOpLoopState nextState = new TeleOpLoopState(
                accumulator.isAutoParking,
                accumulator.autoParkStartTime,
                accumulator.autoAlliance,
                accumulator.shootingPowerMode,
                accumulator.shotgunPowerLatch
        );
        return new TeleOpIterationResult(loopContext.withState(nextState), traceState);
    }
}
