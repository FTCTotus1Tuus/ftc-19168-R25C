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
    public final TeleOpLoopConfig config;

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
            TeleOpLoopConfig config
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
        this.config = config;
    }

    public static TeleOpLoopDependencies create(
            TeleOpCoordinatorSet coordinators,
            TeleOpLoopRuntimeBindings runtime,
            TeleOpLoopConfig config
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
                runtime.localizationService,
                runtime.follower,
                runtime.intakeFSM,
                runtime.shotgunFSM,
                runtime.shootingFSM,
                runtime.turretFSM,
                runtime.gateFSM,
                runtime.telemetry,
                config
        );
    }
}

