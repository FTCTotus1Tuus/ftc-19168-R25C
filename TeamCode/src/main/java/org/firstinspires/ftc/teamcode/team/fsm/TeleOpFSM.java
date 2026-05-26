package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;

import android.annotation.SuppressLint;
import org.firstinspires.ftc.teamcode.team.config.AutoConfig;
import org.firstinspires.ftc.teamcode.team.config.DriveConfig;
import org.firstinspires.ftc.teamcode.team.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.team.config.VisionConfig;
import org.firstinspires.ftc.teamcode.team.core.AutoParkCoordinator;
import org.firstinspires.ftc.teamcode.team.core.DriveControlCoordinator;
import org.firstinspires.ftc.teamcode.team.core.DriverOneBindings;
import org.firstinspires.ftc.teamcode.team.core.DriverTwoBindings;
import org.firstinspires.ftc.teamcode.team.core.IntakeCoordinator;
import org.firstinspires.ftc.teamcode.team.core.OdometryResetCoordinator;
import org.firstinspires.ftc.teamcode.team.core.ShooterPowerCoordinator;
import org.firstinspires.ftc.teamcode.team.core.ShootingCoordinator;
import org.firstinspires.ftc.teamcode.team.core.TeleOpInputMapper;
import org.firstinspires.ftc.teamcode.team.core.TeleOpLoopCoordinator;
import org.firstinspires.ftc.teamcode.team.core.TeleOpStatusCoordinator;
import org.firstinspires.ftc.teamcode.team.core.TeleOpTelemetryCoordinator;
import org.firstinspires.ftc.teamcode.team.core.TurretCoordinator;
import org.firstinspires.ftc.teamcode.team.core.TurretModeCoordinator;
import org.firstinspires.ftc.teamcode.team.core.TurretVisionCoordinator;
import org.firstinspires.ftc.teamcode.team.services.LocalizationService;

@TeleOp(name = "TeleopFSM", group = "DriverControl")
@Config
@Configurable
public class TeleOpFSM extends DarienOpModeFSM {

    // VARIABLES
    private ShotgunPowerLevel shotgunPowerLatch = ShotgunPowerLevel.OFF;
    private AutoParkCoordinator autoParkCoordinator;
    private DriveControlCoordinator driveControlCoordinator;
    private IntakeCoordinator intakeCoordinator;
    private OdometryResetCoordinator odometryResetCoordinator;
    private ShooterPowerCoordinator shooterPowerCoordinator;
    private ShootingCoordinator shootingCoordinator;
    private TeleOpInputMapper inputMapper;
    private TeleOpLoopCoordinator loopCoordinator;
    private TeleOpStatusCoordinator statusCoordinator;
    private TeleOpTelemetryCoordinator telemetryCoordinator;
    private TurretCoordinator turretCoordinator;
    private TurretModeCoordinator turretModeCoordinator;
    private TurretVisionCoordinator turretVisionCoordinator;


    // Turret fallback tracking
    // AUTO-PARK STATE
    private boolean isAutoParking = false;
    private double autoParkStartTime = 0;
    @Override
    public void initControls() {
        super.initControls();
        gateFSM.close();
        turretFSM.center(); // set to center position
        autoParkCoordinator = new AutoParkCoordinator();
        driveControlCoordinator = new DriveControlCoordinator();
        intakeCoordinator = new IntakeCoordinator(intakeFSM);
        odometryResetCoordinator = new OdometryResetCoordinator();
        shooterPowerCoordinator = new ShooterPowerCoordinator();
        shootingCoordinator = new ShootingCoordinator(shootingFSM, intakeFSM, gateFSM);
        inputMapper = new TeleOpInputMapper();
        loopCoordinator = new TeleOpLoopCoordinator();
        statusCoordinator = new TeleOpStatusCoordinator();
        telemetryCoordinator = new TeleOpTelemetryCoordinator();
        turretCoordinator = new TurretCoordinator(turretFSM);
        turretModeCoordinator = new TurretModeCoordinator(turretFSM);
        turretVisionCoordinator = new TurretVisionCoordinator(
                aprilTagService,
                turretFSM,
                VisionConfig.APRILTAG_ID_GOAL_BLUE,
                VisionConfig.APRILTAG_ID_GOAL_RED,
                VisionConfig.CAMERA_FALLBACK_TIMEOUT_MS
        );
    }


    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        initControls();
        tp = new TelemetryPacket();
        dash = FtcDashboard.getInstance();

        String autoAlliance = preferencesService.getAutoAlliance("UNKNOWN");

        // Set align color based on saved color from auto
        turretVisionCoordinator.setAlliance(autoAlliance);

        LocalizationService.SeedResult seedResult = localizationService.seedTeleOpPose(
                autoAlliance,
                preferencesService,
                AutoConfig.HUMAN_PLAYER_RED_X,
                AutoConfig.HUMAN_PLAYER_RED_Y,
                AutoConfig.HUMAN_PLAYER_BLUE_X,
                AutoConfig.HUMAN_PLAYER_BLUE_Y,
                DriveConfig.ROBOT_CENTER_OFFSET_X,
                DriveConfig.ROBOT_CENTER_OFFSET_Y
        );

        if (seedResult.loadedFromAuto) {
            telemetry.addLine("=== ODOMETRY LOADED FROM AUTO ===");
            telemetry.addData("Loaded Position", String.format("X=%.1f, Y=%.1f, H=%.1f°",
                                                               seedResult.x, seedResult.y, Math.toDegrees(seedResult.headingRad)));
        } else {
            telemetry.addLine("=== NO AUTO DATA - DEFAULT POSITION ===");
            telemetry.addData("Default Position", String.format("X=%.1f, Y=%.1f, H=%.1f°",
                                                                seedResult.x, seedResult.y, Math.toDegrees(seedResult.headingRad)));
        }

        waitForStart();
        if (isStopRequested()) return;
        //Start
        follower.startTeleopDrive(true);
        follower.update();

        while (this.opModeIsActive() && !isStopRequested()) {

            // Snapshot gamepad inputs once per loop so mapping stays centralized and traceable.
            DriverOneBindings driverOne = inputMapper.mapDriverOne(gamepad1);
            DriverTwoBindings driverTwo = inputMapper.mapDriverTwo(gamepad2);

            // -----------------
            // ALWAYS RUN
            // -----------------

            TeleOpLoopCoordinator.AlwaysRunResult alwaysRunResult = loopCoordinator.runAlwaysPhase(
                    isAutoParking,
                    autoParkStartTime,
                    shotgunPowerLatch,
                    driverOne,
                    getRuntime(),
                    autoAlliance,
                    autoParkCoordinator,
                    driveControlCoordinator,
                    intakeCoordinator,
                    shootingCoordinator,
                    turretVisionCoordinator,
                    gateFSM,
                    turretFSM,
                    follower,
                    telemetry,
                    AutoConfig.AUTO_PARK_STICK_DEADZONE,
                    AutoConfig.AUTO_PARK_TIMEOUT,
                    DriveConfig.DRIVE_DEADZONE,
                    DriveConfig.INPUT_EXPONENT,
                    DriveConfig.SPEED_SCALE,
                    DriveConfig.SPEED_SCALE_TURN,
                    DriveConfig.ROTATION_SCALE
            );
            isAutoParking = alwaysRunResult.isAutoParking;
            autoParkStartTime = alwaysRunResult.autoParkStartTime;
            shotgunPowerLatch = alwaysRunResult.shotgunPowerLatch;
            double robotX = alwaysRunResult.pose.x;
            double robotY = alwaysRunResult.pose.y;
            double robotHeadingRadians = alwaysRunResult.pose.headingRadians;

            // -----------------
            // GAMEPAD1 CONTROLS
            // -----------------

            TeleOpLoopCoordinator.DriverOnePhaseResult driverOnePhaseResult = loopCoordinator.runDriverOnePhase(
                    isAutoParking,
                    autoParkStartTime,
                    shotgunPowerLatch,
                    autoAlliance,
                    driverOne,
                    driverTwo,
                    getRuntime(),
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
                    AutoConfig.PARK_RED_X,
                    AutoConfig.PARK_RED_Y,
                    AutoConfig.PARK_RED_H_DEG,
                    AutoConfig.PARK_BLUE_X,
                    AutoConfig.PARK_BLUE_Y,
                    AutoConfig.PARK_BLUE_H_DEG,
                    AutoConfig.AUTO_PARK_POWER,
                    AutoConfig.HUMAN_PLAYER_RED_X,
                    AutoConfig.HUMAN_PLAYER_RED_Y,
                    AutoConfig.HUMAN_PLAYER_BLUE_X,
                    AutoConfig.HUMAN_PLAYER_BLUE_Y,
                    DriveConfig.ROBOT_CENTER_OFFSET_X,
                    DriveConfig.ROBOT_CENTER_OFFSET_Y,
                    ShooterConfig.SHOOT_POWER_SELECT_STICK_THRESHOLD
            );
            isAutoParking = driverOnePhaseResult.isAutoParking;
            autoParkStartTime = driverOnePhaseResult.autoParkStartTime;
            shotgunPowerLatch = driverOnePhaseResult.shotgunPowerLatch;

            // -----------------
            // GAMEPAD2 CONTROLS
            // -----------------

            TeleOpLoopCoordinator.DriverTwoPhaseResult driverTwoPhaseResult = loopCoordinator.runDriverTwoPhase(
                    autoAlliance,
                    shootingPowerMode,
                    shotgunPowerLatch,
                    driverTwo,
                    alwaysRunResult.pose,
                    getRuntime(),
                    turretVisionCoordinator,
                    turretModeCoordinator,
                    turretCoordinator,
                    shooterPowerCoordinator,
                    shotgunFSM,
                    telemetry,
                    ShooterConfig.SHOOTING_POWER_ODOMETRY_Y_THRESHOLD,
                    ShooterConfig.SHOOT_POWER_SELECT_STICK_THRESHOLD,
                    ShooterConfig.SHOT_GUN_POWER_UP_RPM,
                    ShooterConfig.SHOT_GUN_POWER_UP_FAR_RPM_TELEOP
            );
            autoAlliance = driverTwoPhaseResult.autoAlliance;
            shootingPowerMode = driverTwoPhaseResult.shootingPowerMode;
            shotgunPowerLatch = driverTwoPhaseResult.shotgunPowerLatch;
            TeleOpStatusCoordinator.TraceState traceState = statusCoordinator.publishStatus(
                    telemetry,
                    telemetryCoordinator,
                    turretVisionCoordinator,
                    gateFSM,
                    intakeFSM,
                    shootingFSM,
                    turretFSM,
                    shootingPowerMode.toString(),
                    shotgunPowerLatch.toString(),
                    ejectionMotor.getVelocity() * 60 / DriveConfig.TICKS_PER_ROTATION,
                    ejectionMotor.getPower(),
                    ejectionMotor.getVelocity(),
                    autoAlliance,
                    robotX,
                    robotY,
                    robotHeadingRadians,
                    isAutoParking,
                    turretVisionCoordinator.getTargetGoalTagId(),
                    AutoConfig.PARK_RED_X,
                    AutoConfig.PARK_RED_Y,
                    AutoConfig.PARK_BLUE_X,
                    AutoConfig.PARK_BLUE_Y,
                    AutoConfig.AUTO_PARK_TIMEOUT,
                    autoParkStartTime,
                    getRuntime()
            );
            addTraceTelemetry("TeleOp", traceState.state, traceState.stateTimerSec);

            telemetry.update();
        } //while opModeIsActive

        stopRobot();
    } //runOpMode


} //TeleOpFSM class
