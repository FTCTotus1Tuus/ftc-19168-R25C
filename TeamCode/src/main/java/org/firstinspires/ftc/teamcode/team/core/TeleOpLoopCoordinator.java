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

    public static class LoopDependencies {
        public final AutoParkCoordinator autoParkCoordinator;
        public final DriveControlCoordinator driveControlCoordinator;
        public final IntakeCoordinator intakeCoordinator;
        public final ShootingCoordinator shootingCoordinator;
        public final OdometryResetCoordinator odometryResetCoordinator;
        public final ShooterPowerCoordinator shooterPowerCoordinator;
        public final TurretVisionCoordinator turretVisionCoordinator;
        public final TurretModeCoordinator turretModeCoordinator;
        public final TurretCoordinator turretCoordinator;
        public final TeleOpStatusCoordinator statusCoordinator;
        public final TeleOpTelemetryCoordinator telemetryCoordinator;
        public final LocalizationService localizationService;
        public final Follower follower;
        public final IntakeFSM intakeFSM;
        public final ShotgunFSM shotgunFSM;
        public final ShootingFSM shootingFSM;
        public final TurretFSM turretFSM;
        public final GateFSM gateFSM;
        public final Telemetry telemetry;
        public final double autoParkStickDeadzone;
        public final double autoParkTimeout;
        public final double driveDeadzone;
        public final double inputExponent;
        public final double speedScale;
        public final double speedScaleTurn;
        public final double rotationScale;
        public final double parkRedX;
        public final double parkRedY;
        public final double parkRedHeadingDeg;
        public final double parkBlueX;
        public final double parkBlueY;
        public final double parkBlueHeadingDeg;
        public final double autoParkPower;
        public final double humanPlayerRedX;
        public final double humanPlayerRedY;
        public final double humanPlayerBlueX;
        public final double humanPlayerBlueY;
        public final double robotCenterOffsetX;
        public final double robotCenterOffsetY;
        public final double shootingPowerOdometryYThreshold;
        public final double shootPowerSelectStickThreshold;
        public final double closeRpm;
        public final double farRpm;

        public LoopDependencies(
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
                double farRpm
        ) {
            this.autoParkCoordinator = autoParkCoordinator;
            this.driveControlCoordinator = driveControlCoordinator;
            this.intakeCoordinator = intakeCoordinator;
            this.shootingCoordinator = shootingCoordinator;
            this.odometryResetCoordinator = odometryResetCoordinator;
            this.shooterPowerCoordinator = shooterPowerCoordinator;
            this.turretVisionCoordinator = turretVisionCoordinator;
            this.turretModeCoordinator = turretModeCoordinator;
            this.turretCoordinator = turretCoordinator;
            this.statusCoordinator = statusCoordinator;
            this.telemetryCoordinator = telemetryCoordinator;
            this.localizationService = localizationService;
            this.follower = follower;
            this.intakeFSM = intakeFSM;
            this.shotgunFSM = shotgunFSM;
            this.shootingFSM = shootingFSM;
            this.turretFSM = turretFSM;
            this.gateFSM = gateFSM;
            this.telemetry = telemetry;
            this.autoParkStickDeadzone = autoParkStickDeadzone;
            this.autoParkTimeout = autoParkTimeout;
            this.driveDeadzone = driveDeadzone;
            this.inputExponent = inputExponent;
            this.speedScale = speedScale;
            this.speedScaleTurn = speedScaleTurn;
            this.rotationScale = rotationScale;
            this.parkRedX = parkRedX;
            this.parkRedY = parkRedY;
            this.parkRedHeadingDeg = parkRedHeadingDeg;
            this.parkBlueX = parkBlueX;
            this.parkBlueY = parkBlueY;
            this.parkBlueHeadingDeg = parkBlueHeadingDeg;
            this.autoParkPower = autoParkPower;
            this.humanPlayerRedX = humanPlayerRedX;
            this.humanPlayerRedY = humanPlayerRedY;
            this.humanPlayerBlueX = humanPlayerBlueX;
            this.humanPlayerBlueY = humanPlayerBlueY;
            this.robotCenterOffsetX = robotCenterOffsetX;
            this.robotCenterOffsetY = robotCenterOffsetY;
            this.shootingPowerOdometryYThreshold = shootingPowerOdometryYThreshold;
            this.shootPowerSelectStickThreshold = shootPowerSelectStickThreshold;
            this.closeRpm = closeRpm;
            this.farRpm = farRpm;
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
            LoopDependencies deps,
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
                deps.autoParkCoordinator,
                deps.driveControlCoordinator,
                deps.intakeCoordinator,
                deps.shootingCoordinator,
                deps.turretVisionCoordinator,
                deps.gateFSM,
                deps.turretFSM,
                deps.follower,
                deps.telemetry,
                deps.autoParkStickDeadzone,
                deps.autoParkTimeout,
                deps.driveDeadzone,
                deps.inputExponent,
                deps.speedScale,
                deps.speedScaleTurn,
                deps.rotationScale
        );

        DriverOnePhaseResult driverOnePhaseResult = runDriverOnePhase(
                alwaysRunResult.isAutoParking,
                alwaysRunResult.autoParkStartTime,
                alwaysRunResult.shotgunPowerLatch,
                loopState.autoAlliance,
                driverOne,
                driverTwo,
                currentTime,
                deps.autoParkCoordinator,
                deps.intakeCoordinator,
                deps.shootingCoordinator,
                deps.odometryResetCoordinator,
                deps.localizationService,
                deps.follower,
                deps.intakeFSM,
                deps.shotgunFSM,
                deps.shootingFSM,
                deps.turretFSM,
                deps.gateFSM,
                deps.telemetry,
                deps.parkRedX,
                deps.parkRedY,
                deps.parkRedHeadingDeg,
                deps.parkBlueX,
                deps.parkBlueY,
                deps.parkBlueHeadingDeg,
                deps.autoParkPower,
                deps.humanPlayerRedX,
                deps.humanPlayerRedY,
                deps.humanPlayerBlueX,
                deps.humanPlayerBlueY,
                deps.robotCenterOffsetX,
                deps.robotCenterOffsetY,
                deps.shootPowerSelectStickThreshold
        );

        DriverTwoPhaseResult driverTwoPhaseResult = runDriverTwoPhase(
                loopState.autoAlliance,
                loopState.shootingPowerMode,
                driverOnePhaseResult.shotgunPowerLatch,
                driverTwo,
                alwaysRunResult.pose,
                currentTime,
                deps.turretVisionCoordinator,
                deps.turretModeCoordinator,
                deps.turretCoordinator,
                deps.shooterPowerCoordinator,
                deps.shotgunFSM,
                deps.telemetry,
                deps.shootingPowerOdometryYThreshold,
                deps.shootPowerSelectStickThreshold,
                deps.closeRpm,
                deps.farRpm
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
                deps.parkRedX,
                deps.parkRedY,
                deps.parkBlueX,
                deps.parkBlueY,
                deps.autoParkTimeout,
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

