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

    private static class AlwaysRunResult {
        public final boolean isAutoParking;
        public final double autoParkStartTime;
        public final DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch;
        public final PoseSnapshot pose;

        public AlwaysRunResult(
                boolean isAutoParking,
                double autoParkStartTime,
                DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch,
                PoseSnapshot pose
        ) {
            this.isAutoParking = isAutoParking;
            this.autoParkStartTime = autoParkStartTime;
            this.shotgunPowerLatch = shotgunPowerLatch;
            this.pose = pose;
        }
    }

    private static class DriverOnePhaseResult {
        public final boolean isAutoParking;
        public final double autoParkStartTime;
        public final DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch;

        public DriverOnePhaseResult(
                boolean isAutoParking,
                double autoParkStartTime,
                DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch
        ) {
            this.isAutoParking = isAutoParking;
            this.autoParkStartTime = autoParkStartTime;
            this.shotgunPowerLatch = shotgunPowerLatch;
        }
    }

    private static class DriverTwoPhaseResult {
        public final String autoAlliance;
        public final DarienOpModeFSM.ShootingPowerModes shootingPowerMode;
        public final DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch;

        public DriverTwoPhaseResult(
                String autoAlliance,
                DarienOpModeFSM.ShootingPowerModes shootingPowerMode,
                DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch
        ) {
            this.autoAlliance = autoAlliance;
            this.shootingPowerMode = shootingPowerMode;
            this.shotgunPowerLatch = shotgunPowerLatch;
        }
    }


    private AlwaysRunResult runAlwaysPhase(
            boolean isAutoParking,
            double autoParkStartTime,
            DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch,
            DriverOneBindings driverOne,
            double currentTime,
            String autoAlliance,
            TeleOpLoopDependencies deps
    ) {
        AutoParkCoordinator.AutoParkResult progressResult = deps.autoParkCoordinator.updateAutoParkProgress(
                isAutoParking,
                autoParkStartTime,
                currentTime,
                driverOne.driveStrafeAxis,
                driverOne.driveForwardAxis,
                driverOne.driveTurnAxis,
                deps.config.autoParkStickDeadzone,
                deps.config.autoParkTimeout,
                deps.follower,
                deps.telemetry,
                shotgunPowerLatch
        );
        isAutoParking = progressResult.isAutoParking;
        autoParkStartTime = progressResult.autoParkStartTime;
        shotgunPowerLatch = progressResult.shotgunPowerLatch;

        deps.driveControlCoordinator.applyTeleOpDrive(
                isAutoParking,
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

        PoseSnapshot pose = new PoseSnapshot(
                deps.follower.getPose().getX(),
                deps.follower.getPose().getY(),
                deps.follower.getPose().getHeading()
        );

        deps.turretVisionCoordinator.updateCameraControl(
                currentTime,
                autoAlliance,
                pose.x,
                pose.y,
                pose.headingRadians,
                deps.telemetry
        );

        return new AlwaysRunResult(isAutoParking, autoParkStartTime, shotgunPowerLatch, pose);
    }

    private DriverOnePhaseResult runDriverOnePhase(
            boolean isAutoParking,
            double autoParkStartTime,
            DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch,
            String autoAlliance,
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
                isAutoParking,
                currentTime,
                autoParkStartTime,
                autoAlliance,
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
                shotgunPowerLatch
        );
        isAutoParking = startResult.isAutoParking;
        autoParkStartTime = startResult.autoParkStartTime;
        shotgunPowerLatch = startResult.shotgunPowerLatch;

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
                autoAlliance,
                deps.config.humanPlayerRedX,
                deps.config.humanPlayerRedY,
                deps.config.humanPlayerBlueX,
                deps.config.humanPlayerBlueY,
                deps.config.robotCenterOffsetX,
                deps.config.robotCenterOffsetY,
                deps.localizationService,
                deps.telemetry
        );

        return new DriverOnePhaseResult(isAutoParking, autoParkStartTime, shotgunPowerLatch);
    }

    private DriverTwoPhaseResult runDriverTwoPhase(
            String autoAlliance,
            DarienOpModeFSM.ShootingPowerModes shootingPowerMode,
            DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch,
            DriverTwoBindings driverTwo,
            PoseSnapshot pose,
            double currentTime,
            TeleOpLoopDependencies deps
    ) {
        TurretVisionCoordinator.AllianceSwitchResult allianceSwitchResult = deps.turretVisionCoordinator.handleAllianceButtons(
                driverTwo.setAllianceRedRequested,
                driverTwo.setAllianceBlueRequested,
                autoAlliance,
                deps.telemetry
        );
        autoAlliance = allianceSwitchResult.alliance;

        TurretModeCoordinator.ModeSwitchResult modeSwitchResult = deps.turretModeCoordinator.handleModeSwitches(
                driverTwo.odometryModeRequested,
                driverTwo.cameraModeRequested,
                shootingPowerMode
        );
        shootingPowerMode = modeSwitchResult.shootingPowerMode;
        if (modeSwitchResult.shouldStartGoalReading) {
            deps.turretVisionCoordinator.startReadingGoalId(currentTime);
        }

        deps.turretCoordinator.applyManualOrOdometryControl(
                autoAlliance,
                driverTwo.turretManualAxis,
                driverTwo.turretSpeedTrigger,
                driverTwo.turretCenterRequested,
                pose.x,
                pose.y,
                pose.headingRadians
        );

        ShooterPowerCoordinator.PowerState powerState = deps.shooterPowerCoordinator.computePowerState(
                shootingPowerMode,
                shotgunPowerLatch,
                pose.y,
                deps.config.shootingPowerOdometryYThreshold,
                driverTwo.shootingStickY,
                deps.config.shootPowerSelectStickThreshold,
                driverTwo.toggleShotgunPowerLatchRequested,
                driverTwo.forceShotgunLowPowerRequested
        );
        shootingPowerMode = powerState.mode;
        shotgunPowerLatch = powerState.latch;

        deps.shooterPowerCoordinator.applyRequestedPower(
                deps.shotgunFSM,
                shotgunPowerLatch,
                deps.config.closeRpm,
                deps.config.farRpm,
                deps.telemetry
        );

        return new DriverTwoPhaseResult(autoAlliance, shootingPowerMode, shotgunPowerLatch);
    }

    public TeleOpIterationResult runLoopIteration(
            TeleOpLoopContext loopContext,
            DriverOneBindings driverOne,
            DriverTwoBindings driverTwo,
            double currentTime,
            double ejectionMotorRpm,
            double ejectionMotorPower,
            double ejectionMotorVelocity
    ) {
        TeleOpLoopState loopState = loopContext.state;
        TeleOpLoopDependencies deps = loopContext.dependencies;

        AlwaysRunResult alwaysRunResult = runAlwaysPhase(
                loopState.isAutoParking,
                loopState.autoParkStartTime,
                loopState.shotgunPowerLatch,
                driverOne,
                currentTime,
                loopState.autoAlliance,
                deps
        );

        DriverOnePhaseResult driverOnePhaseResult = runDriverOnePhase(
                alwaysRunResult.isAutoParking,
                alwaysRunResult.autoParkStartTime,
                alwaysRunResult.shotgunPowerLatch,
                loopState.autoAlliance,
                driverOne,
                driverTwo,
                currentTime,
                deps
        );

        DriverTwoPhaseResult driverTwoPhaseResult = runDriverTwoPhase(
                loopState.autoAlliance,
                loopState.shootingPowerMode,
                driverOnePhaseResult.shotgunPowerLatch,
                driverTwo,
                alwaysRunResult.pose,
                currentTime,
                deps
        );

        TeleOpStatusCoordinator.TraceState traceState = deps.statusCoordinator.publishStatus(
                deps.telemetry,
                deps.telemetryCoordinator,
                deps.turretVisionCoordinator,
                deps.gateFSM,
                deps.intakeFSM,
                deps.shootingFSM,
                deps.turretFSM,
                driverTwoPhaseResult.shootingPowerMode.toString(),
                driverTwoPhaseResult.shotgunPowerLatch.toString(),
                ejectionMotorRpm,
                ejectionMotorPower,
                ejectionMotorVelocity,
                driverTwoPhaseResult.autoAlliance,
                alwaysRunResult.pose.x,
                alwaysRunResult.pose.y,
                alwaysRunResult.pose.headingRadians,
                driverOnePhaseResult.isAutoParking,
                deps.turretVisionCoordinator.getTargetGoalTagId(),
                deps.config.parkRedX,
                deps.config.parkRedY,
                deps.config.parkBlueX,
                deps.config.parkBlueY,
                deps.config.autoParkTimeout,
                driverOnePhaseResult.autoParkStartTime,
                currentTime
        );

        TeleOpLoopState nextState = new TeleOpLoopState(
                driverOnePhaseResult.isAutoParking,
                driverOnePhaseResult.autoParkStartTime,
                driverTwoPhaseResult.autoAlliance,
                driverTwoPhaseResult.shootingPowerMode,
                driverTwoPhaseResult.shotgunPowerLatch
        );
        return new TeleOpIterationResult(loopContext.withState(nextState), traceState);
    }
}
