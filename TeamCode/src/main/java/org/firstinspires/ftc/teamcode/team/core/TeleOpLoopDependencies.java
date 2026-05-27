package org.firstinspires.ftc.teamcode.team.core;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team.fsm.GateFSM;
import org.firstinspires.ftc.teamcode.team.fsm.IntakeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootingFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShotgunFSM;
import org.firstinspires.ftc.teamcode.team.fsm.TurretFSM;
import org.firstinspires.ftc.teamcode.team.services.LocalizationService;

/**
 * Stable TeleOp loop dependencies and constants reused across iterations.
 */
public class TeleOpLoopDependencies {
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

    public TeleOpLoopDependencies(
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

    public static TeleOpLoopDependencies create(
            TeleOpCoordinatorSet coordinators,
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
        return new TeleOpLoopDependencies(
                coordinators.autoParkCoordinator,
                coordinators.driveControlCoordinator,
                coordinators.intakeCoordinator,
                coordinators.shootingCoordinator,
                coordinators.odometryResetCoordinator,
                coordinators.shooterPowerCoordinator,
                coordinators.turretVisionCoordinator,
                coordinators.turretModeCoordinator,
                coordinators.turretCoordinator,
                coordinators.statusCoordinator,
                coordinators.telemetryCoordinator,
                localizationService,
                follower,
                intakeFSM,
                shotgunFSM,
                shootingFSM,
                turretFSM,
                gateFSM,
                telemetry,
                autoParkStickDeadzone,
                autoParkTimeout,
                driveDeadzone,
                inputExponent,
                speedScale,
                speedScaleTurn,
                rotationScale,
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
                shootingPowerOdometryYThreshold,
                shootPowerSelectStickThreshold,
                closeRpm,
                farRpm
        );
    }
}

