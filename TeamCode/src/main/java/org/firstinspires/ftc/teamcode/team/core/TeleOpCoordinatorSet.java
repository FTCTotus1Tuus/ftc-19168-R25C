package org.firstinspires.ftc.teamcode.team.core;

/**
 * Bundles TeleOp coordinator instances so the OpMode shell owns one reference.
 */
public class TeleOpCoordinatorSet {
    public final AutoParkCoordinator autoParkCoordinator;
    public final DriveControlCoordinator driveControlCoordinator;
    public final IntakeCoordinator intakeCoordinator;
    public final OdometryResetCoordinator odometryResetCoordinator;
    public final ShooterPowerCoordinator shooterPowerCoordinator;
    public final ShootingCoordinator shootingCoordinator;
    public final TeleOpInitializationCoordinator initializationCoordinator;
    public final TeleOpInputMapper inputMapper;
    public final TeleOpLoopCoordinator loopCoordinator;
    public final TeleOpStatusCoordinator statusCoordinator;
    public final TeleOpTelemetryCoordinator telemetryCoordinator;
    public final TurretCoordinator turretCoordinator;
    public final TurretModeCoordinator turretModeCoordinator;
    public final TurretVisionCoordinator turretVisionCoordinator;

    public TeleOpCoordinatorSet(
            AutoParkCoordinator autoParkCoordinator,
            DriveControlCoordinator driveControlCoordinator,
            IntakeCoordinator intakeCoordinator,
            OdometryResetCoordinator odometryResetCoordinator,
            ShooterPowerCoordinator shooterPowerCoordinator,
            ShootingCoordinator shootingCoordinator,
            TeleOpInitializationCoordinator initializationCoordinator,
            TeleOpInputMapper inputMapper,
            TeleOpLoopCoordinator loopCoordinator,
            TeleOpStatusCoordinator statusCoordinator,
            TeleOpTelemetryCoordinator telemetryCoordinator,
            TurretCoordinator turretCoordinator,
            TurretModeCoordinator turretModeCoordinator,
            TurretVisionCoordinator turretVisionCoordinator
    ) {
        this.autoParkCoordinator = autoParkCoordinator;
        this.driveControlCoordinator = driveControlCoordinator;
        this.intakeCoordinator = intakeCoordinator;
        this.odometryResetCoordinator = odometryResetCoordinator;
        this.shooterPowerCoordinator = shooterPowerCoordinator;
        this.shootingCoordinator = shootingCoordinator;
        this.initializationCoordinator = initializationCoordinator;
        this.inputMapper = inputMapper;
        this.loopCoordinator = loopCoordinator;
        this.statusCoordinator = statusCoordinator;
        this.telemetryCoordinator = telemetryCoordinator;
        this.turretCoordinator = turretCoordinator;
        this.turretModeCoordinator = turretModeCoordinator;
        this.turretVisionCoordinator = turretVisionCoordinator;
    }
}

