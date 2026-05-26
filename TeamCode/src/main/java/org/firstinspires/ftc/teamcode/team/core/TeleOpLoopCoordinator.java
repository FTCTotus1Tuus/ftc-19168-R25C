package org.firstinspires.ftc.teamcode.team.core;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.GateFSM;
import org.firstinspires.ftc.teamcode.team.fsm.IntakeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootingFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShotgunFSM;
import org.firstinspires.ftc.teamcode.team.fsm.TurretFSM;
import org.firstinspires.ftc.teamcode.team.services.LocalizationService;

/**
 * Coordinates high-level TeleOp loop phases while preserving existing coordinator behavior.
 */
public class TeleOpLoopCoordinator {

    public static class PoseSnapshot {
        public final double x;
        public final double y;
        public final double headingRadians;

        public PoseSnapshot(double x, double y, double headingRadians) {
            this.x = x;
            this.y = y;
            this.headingRadians = headingRadians;
        }
    }

    public static class AlwaysRunResult {
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

    public static class DriverOnePhaseResult {
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

    public static class DriverTwoPhaseResult {
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

    public static class LoopState {
        public final boolean isAutoParking;
        public final double autoParkStartTime;
        public final String autoAlliance;
        public final DarienOpModeFSM.ShootingPowerModes shootingPowerMode;
        public final DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch;

        public LoopState(
                boolean isAutoParking,
                double autoParkStartTime,
                String autoAlliance,
                DarienOpModeFSM.ShootingPowerModes shootingPowerMode,
                DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch
        ) {
            this.isAutoParking = isAutoParking;
            this.autoParkStartTime = autoParkStartTime;
            this.autoAlliance = autoAlliance;
            this.shootingPowerMode = shootingPowerMode;
            this.shotgunPowerLatch = shotgunPowerLatch;
        }
    }

    public static class IterationResult {
        public final LoopState state;
        public final TeleOpStatusCoordinator.TraceState traceState;

        public IterationResult(LoopState state, TeleOpStatusCoordinator.TraceState traceState) {
            this.state = state;
            this.traceState = traceState;
        }
    }

    public AlwaysRunResult runAlwaysPhase(
            boolean isAutoParking,
            double autoParkStartTime,
            DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch,
            DriverOneBindings driverOne,
            double currentTime,
            String autoAlliance,
            AutoParkCoordinator autoParkCoordinator,
            DriveControlCoordinator driveControlCoordinator,
            IntakeCoordinator intakeCoordinator,
            ShootingCoordinator shootingCoordinator,
            TurretVisionCoordinator turretVisionCoordinator,
            GateFSM gateFSM,
            TurretFSM turretFSM,
            Follower follower,
            Telemetry telemetry,
            double autoParkStickDeadzone,
            double autoParkTimeout,
            double driveDeadzone,
            double inputExponent,
            double speedScale,
            double speedScaleTurn,
            double rotationScale
    ) {
        AutoParkCoordinator.AutoParkResult progressResult = autoParkCoordinator.updateAutoParkProgress(
                isAutoParking,
                autoParkStartTime,
                currentTime,
                driverOne.driveStrafeAxis,
                driverOne.driveForwardAxis,
                driverOne.driveTurnAxis,
                autoParkStickDeadzone,
                autoParkTimeout,
                follower,
                telemetry,
                shotgunPowerLatch
        );
        isAutoParking = progressResult.isAutoParking;
        autoParkStartTime = progressResult.autoParkStartTime;
        shotgunPowerLatch = progressResult.shotgunPowerLatch;

        driveControlCoordinator.applyTeleOpDrive(
                isAutoParking,
                driverOne.driveForwardAxis,
                driverOne.driveStrafeAxis,
                driverOne.driveTurnAxis,
                driveDeadzone,
                inputExponent,
                speedScale,
                speedScaleTurn,
                rotationScale,
                follower
        );

        follower.update();

        gateFSM.update(currentTime, telemetry);
        turretFSM.update(currentTime, telemetry);

        intakeCoordinator.updateActiveIntake(currentTime, telemetry);
        shootingCoordinator.updateActiveSequence(currentTime, telemetry);

        PoseSnapshot pose = new PoseSnapshot(
                follower.getPose().getX(),
                follower.getPose().getY(),
                follower.getPose().getHeading()
        );

        turretVisionCoordinator.updateCameraControl(
                currentTime,
                autoAlliance,
                pose.x,
                pose.y,
                pose.headingRadians,
                telemetry
        );

        return new AlwaysRunResult(isAutoParking, autoParkStartTime, shotgunPowerLatch, pose);
    }

    public DriverOnePhaseResult runDriverOnePhase(
            boolean isAutoParking,
            double autoParkStartTime,
            DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch,
            String autoAlliance,
            DriverOneBindings driverOne,
            DriverTwoBindings driverTwo,
            double currentTime,
            AutoParkCoordinator autoParkCoordinator,
            IntakeCoordinator intakeCoordinator,
            ShootingCoordinator shootingCoordinator,
            OdometryResetCoordinator odometryResetCoordinator,
            LocalizationService localizationService,
            Follower follower,
            IntakeFSM intakeFSM,
            ShotgunFSM shotgunFSM,
            ShootingFSM shootingFSM,
            TurretFSM turretFSM,
            GateFSM gateFSM,
            Telemetry telemetry,
            double parkRedX,
            double parkRedY,
            double parkRedHeadingDeg,
            double parkBlueX,
            double parkBlueY,
            double parkBlueHeadingDeg,
            double autoParkPower,
            double humanPlayerRedX,
            double humanPlayerRedY,
            double humanPlayerBlueX,
            double humanPlayerBlueY,
            double robotCenterOffsetX,
            double robotCenterOffsetY,
            double shootPowerSelectStickThreshold
    ) {
        intakeCoordinator.handleDriverControls(
                driverOne.intakeRequested,
                driverOne.ejectRequested,
                driverOne.intakeOffRequested
        );

        AutoParkCoordinator.AutoParkResult startResult = autoParkCoordinator.tryStartAutoPark(
                driverOne.autoParkRequested,
                isAutoParking,
                currentTime,
                autoParkStartTime,
                autoAlliance,
                parkRedX,
                parkRedY,
                parkRedHeadingDeg,
                parkBlueX,
                parkBlueY,
                parkBlueHeadingDeg,
                autoParkPower,
                follower,
                intakeFSM,
                shotgunFSM,
                shootingFSM,
                turretFSM,
                gateFSM,
                shotgunPowerLatch
        );
        isAutoParking = startResult.isAutoParking;
        autoParkStartTime = startResult.autoParkStartTime;
        shotgunPowerLatch = startResult.shotgunPowerLatch;

        shootingCoordinator.handleDriverControls(
                currentTime,
                driverTwo.closeGateRequested,
                driverTwo.shootPressed,
                driverTwo.shootReleased,
                driverTwo.shootingStickY,
                shootPowerSelectStickThreshold
        );

        odometryResetCoordinator.tryResetToHumanPlayerPosition(
                driverOne.odometryResetRequested,
                autoAlliance,
                humanPlayerRedX,
                humanPlayerRedY,
                humanPlayerBlueX,
                humanPlayerBlueY,
                robotCenterOffsetX,
                robotCenterOffsetY,
                localizationService,
                telemetry
        );

        return new DriverOnePhaseResult(isAutoParking, autoParkStartTime, shotgunPowerLatch);
    }

    public DriverTwoPhaseResult runDriverTwoPhase(
            String autoAlliance,
            DarienOpModeFSM.ShootingPowerModes shootingPowerMode,
            DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch,
            DriverTwoBindings driverTwo,
            PoseSnapshot pose,
            double currentTime,
            TurretVisionCoordinator turretVisionCoordinator,
            TurretModeCoordinator turretModeCoordinator,
            TurretCoordinator turretCoordinator,
            ShooterPowerCoordinator shooterPowerCoordinator,
            ShotgunFSM shotgunFSM,
            Telemetry telemetry,
            double shootingPowerOdometryYThreshold,
            double shootPowerSelectStickThreshold,
            double closeRpm,
            double farRpm
    ) {
        TurretVisionCoordinator.AllianceSwitchResult allianceSwitchResult = turretVisionCoordinator.handleAllianceButtons(
                driverTwo.setAllianceRedRequested,
                driverTwo.setAllianceBlueRequested,
                autoAlliance,
                telemetry
        );
        autoAlliance = allianceSwitchResult.alliance;

        TurretModeCoordinator.ModeSwitchResult modeSwitchResult = turretModeCoordinator.handleModeSwitches(
                driverTwo.odometryModeRequested,
                driverTwo.cameraModeRequested,
                shootingPowerMode
        );
        shootingPowerMode = modeSwitchResult.shootingPowerMode;
        if (modeSwitchResult.shouldStartGoalReading) {
            turretVisionCoordinator.startReadingGoalId(currentTime);
        }

        turretCoordinator.applyManualOrOdometryControl(
                autoAlliance,
                driverTwo.turretManualAxis,
                driverTwo.turretSpeedTrigger,
                driverTwo.turretCenterRequested,
                pose.x,
                pose.y,
                pose.headingRadians
        );

        ShooterPowerCoordinator.PowerState powerState = shooterPowerCoordinator.computePowerState(
                shootingPowerMode,
                shotgunPowerLatch,
                pose.y,
                shootingPowerOdometryYThreshold,
                driverTwo.shootingStickY,
                shootPowerSelectStickThreshold,
                driverTwo.toggleShotgunPowerLatchRequested,
                driverTwo.forceShotgunLowPowerRequested
        );
        shootingPowerMode = powerState.mode;
        shotgunPowerLatch = powerState.latch;

        shooterPowerCoordinator.applyRequestedPower(
                shotgunFSM,
                shotgunPowerLatch,
                closeRpm,
                farRpm,
                telemetry
        );

        return new DriverTwoPhaseResult(autoAlliance, shootingPowerMode, shotgunPowerLatch);
    }

    public IterationResult runLoopIteration(
            LoopState loopState,
            DriverOneBindings driverOne,
            DriverTwoBindings driverTwo,
            double currentTime,
            AutoParkCoordinator autoParkCoordinator,
            DriveControlCoordinator driveControlCoordinator,
            IntakeCoordinator intakeCoordinator,
            ShootingCoordinator shootingCoordinator,
            OdometryResetCoordinator odometryResetCoordinator,
            ShooterPowerCoordinator shooterPowerCoordinator,
            TurretVisionCoordinator turretVisionCoordinator,
            TurretModeCoordinator turretModeCoordinator,
            TurretCoordinator turretCoordinator,
            TeleOpStatusCoordinator statusCoordinator,
            TeleOpTelemetryCoordinator telemetryCoordinator,
            LocalizationService localizationService,
            Follower follower,
            IntakeFSM intakeFSM,
            ShotgunFSM shotgunFSM,
            ShootingFSM shootingFSM,
            TurretFSM turretFSM,
            GateFSM gateFSM,
            Telemetry telemetry,
            double autoParkStickDeadzone,
            double autoParkTimeout,
            double driveDeadzone,
            double inputExponent,
            double speedScale,
            double speedScaleTurn,
            double rotationScale,
            double parkRedX,
            double parkRedY,
            double parkRedHeadingDeg,
            double parkBlueX,
            double parkBlueY,
            double parkBlueHeadingDeg,
            double autoParkPower,
            double humanPlayerRedX,
            double humanPlayerRedY,
            double humanPlayerBlueX,
            double humanPlayerBlueY,
            double robotCenterOffsetX,
            double robotCenterOffsetY,
            double shootingPowerOdometryYThreshold,
            double shootPowerSelectStickThreshold,
            double closeRpm,
            double farRpm,
            double ejectionMotorRpm,
            double ejectionMotorPower,
            double ejectionMotorVelocity
    ) {
        AlwaysRunResult alwaysRunResult = runAlwaysPhase(
                loopState.isAutoParking,
                loopState.autoParkStartTime,
                loopState.shotgunPowerLatch,
                driverOne,
                currentTime,
                loopState.autoAlliance,
                autoParkCoordinator,
                driveControlCoordinator,
                intakeCoordinator,
                shootingCoordinator,
                turretVisionCoordinator,
                gateFSM,
                turretFSM,
                follower,
                telemetry,
                autoParkStickDeadzone,
                autoParkTimeout,
                driveDeadzone,
                inputExponent,
                speedScale,
                speedScaleTurn,
                rotationScale
        );

        DriverOnePhaseResult driverOnePhaseResult = runDriverOnePhase(
                alwaysRunResult.isAutoParking,
                alwaysRunResult.autoParkStartTime,
                alwaysRunResult.shotgunPowerLatch,
                loopState.autoAlliance,
                driverOne,
                driverTwo,
                currentTime,
                autoParkCoordinator,
                intakeCoordinator,
                shootingCoordinator,
                odometryResetCoordinator,
                localizationService,
                follower,
                intakeFSM,
                shotgunFSM,
                shootingFSM,
                turretFSM,
                gateFSM,
                telemetry,
                parkRedX,
                parkRedY,
                parkRedHeadingDeg,
                parkBlueX,
                parkBlueY,
                parkBlueHeadingDeg,
                autoParkPower,
                humanPlayerRedX,
                humanPlayerRedY,
                humanPlayerBlueX,
                humanPlayerBlueY,
                robotCenterOffsetX,
                robotCenterOffsetY,
                shootPowerSelectStickThreshold
        );

        DriverTwoPhaseResult driverTwoPhaseResult = runDriverTwoPhase(
                loopState.autoAlliance,
                loopState.shootingPowerMode,
                driverOnePhaseResult.shotgunPowerLatch,
                driverTwo,
                alwaysRunResult.pose,
                currentTime,
                turretVisionCoordinator,
                turretModeCoordinator,
                turretCoordinator,
                shooterPowerCoordinator,
                shotgunFSM,
                telemetry,
                shootingPowerOdometryYThreshold,
                shootPowerSelectStickThreshold,
                closeRpm,
                farRpm
        );

        TeleOpStatusCoordinator.TraceState traceState = statusCoordinator.publishStatus(
                telemetry,
                telemetryCoordinator,
                turretVisionCoordinator,
                gateFSM,
                intakeFSM,
                shootingFSM,
                turretFSM,
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
                turretVisionCoordinator.getTargetGoalTagId(),
                parkRedX,
                parkRedY,
                parkBlueX,
                parkBlueY,
                autoParkTimeout,
                driverOnePhaseResult.autoParkStartTime,
                currentTime
        );

        LoopState nextState = new LoopState(
                driverOnePhaseResult.isAutoParking,
                driverOnePhaseResult.autoParkStartTime,
                driverTwoPhaseResult.autoAlliance,
                driverTwoPhaseResult.shootingPowerMode,
                driverTwoPhaseResult.shotgunPowerLatch
        );
        return new IterationResult(nextState, traceState);
    }
}

