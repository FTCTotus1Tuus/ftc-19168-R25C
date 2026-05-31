package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.teamcode.team.config.VisionConfig;
import org.firstinspires.ftc.teamcode.team.fsm.GateFSM;
import org.firstinspires.ftc.teamcode.team.fsm.IntakeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootingFSM;
import org.firstinspires.ftc.teamcode.team.fsm.TurretFSM;
import org.firstinspires.ftc.teamcode.team.services.AprilTagService;

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

    public static TeleOpCoordinatorSet create(
            AprilTagService aprilTagService,
            TurretFSM turretFSM,
            IntakeFSM intakeFSM,
            ShootingFSM shootingFSM,
            GateFSM gateFSM
    ) {
        AutoParkCoordinator autoParkCoordinator = new AutoParkCoordinator();
        DriveControlCoordinator driveControlCoordinator = new DriveControlCoordinator();
        IntakeCoordinator intakeCoordinator = new IntakeCoordinator(intakeFSM);
        OdometryResetCoordinator odometryResetCoordinator = new OdometryResetCoordinator();
        ShooterPowerCoordinator shooterPowerCoordinator = new ShooterPowerCoordinator();
        ShootingCoordinator shootingCoordinator = new ShootingCoordinator(shootingFSM, intakeFSM, gateFSM);
        TeleOpInitializationCoordinator initializationCoordinator = new TeleOpInitializationCoordinator();
        TeleOpInputMapper inputMapper = new TeleOpInputMapper();
        TeleOpLoopCoordinator loopCoordinator = new TeleOpLoopCoordinator();
        TeleOpStatusCoordinator statusCoordinator = new TeleOpStatusCoordinator();
        TeleOpTelemetryCoordinator telemetryCoordinator = new TeleOpTelemetryCoordinator();
        TurretCoordinator turretCoordinator = new TurretCoordinator(turretFSM);
        TurretModeCoordinator turretModeCoordinator = new TurretModeCoordinator(turretFSM);
        TurretVisionCoordinator turretVisionCoordinator = createTurretVisionCoordinator(aprilTagService, turretFSM);

        return new TeleOpCoordinatorSet(
                autoParkCoordinator,
                driveControlCoordinator,
                intakeCoordinator,
                odometryResetCoordinator,
                shooterPowerCoordinator,
                shootingCoordinator,
                initializationCoordinator,
                inputMapper,
                loopCoordinator,
                statusCoordinator,
                telemetryCoordinator,
                turretCoordinator,
                turretModeCoordinator,
                turretVisionCoordinator
        );
    }

    private static TurretVisionCoordinator createTurretVisionCoordinator(
            AprilTagService aprilTagService,
            TurretFSM turretFSM
    ) {
        return new TurretVisionCoordinator(
                aprilTagService,
                turretFSM,
                VisionConfig.APRILTAG_ID_GOAL_BLUE,
                VisionConfig.APRILTAG_ID_GOAL_RED,
                VisionConfig.CAMERA_FALLBACK_TIMEOUT_MS
        );
    }
}

