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
import org.firstinspires.ftc.teamcode.team.core.TeleOpInitializationCoordinator;
import org.firstinspires.ftc.teamcode.team.core.TeleOpInputMapper;
import org.firstinspires.ftc.teamcode.team.core.TeleOpLoopCoordinator;
import org.firstinspires.ftc.teamcode.team.core.TeleOpStatusCoordinator;
import org.firstinspires.ftc.teamcode.team.core.TeleOpTelemetryCoordinator;
import org.firstinspires.ftc.teamcode.team.core.TurretCoordinator;
import org.firstinspires.ftc.teamcode.team.core.TurretModeCoordinator;
import org.firstinspires.ftc.teamcode.team.core.TurretVisionCoordinator;

@TeleOp(name = "TeleopFSM", group = "DriverControl")
@Config
@Configurable
public class TeleOpFSM extends DarienOpModeFSM {

    private AutoParkCoordinator autoParkCoordinator;
    private DriveControlCoordinator driveControlCoordinator;
    private IntakeCoordinator intakeCoordinator;
    private OdometryResetCoordinator odometryResetCoordinator;
    private ShooterPowerCoordinator shooterPowerCoordinator;
    private ShootingCoordinator shootingCoordinator;
    private TeleOpInitializationCoordinator initializationCoordinator;
    private TeleOpInputMapper inputMapper;
    private TeleOpLoopCoordinator loopCoordinator;
    private TeleOpStatusCoordinator statusCoordinator;
    private TeleOpTelemetryCoordinator telemetryCoordinator;
    private TurretCoordinator turretCoordinator;
    private TurretModeCoordinator turretModeCoordinator;
    private TurretVisionCoordinator turretVisionCoordinator;

    @Override
    public void initControls() {
        super.initControls();
        gateFSM.close();
        turretFSM.center(); // set to center position
        initializeCoordinators();
    }

    private void initializeCoordinators() {
        autoParkCoordinator = new AutoParkCoordinator();
        driveControlCoordinator = new DriveControlCoordinator();
        intakeCoordinator = new IntakeCoordinator(intakeFSM);
        odometryResetCoordinator = new OdometryResetCoordinator();
        shooterPowerCoordinator = new ShooterPowerCoordinator();
        shootingCoordinator = new ShootingCoordinator(shootingFSM, intakeFSM, gateFSM);
        initializationCoordinator = new TeleOpInitializationCoordinator();
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

        TeleOpInitializationCoordinator.InitializationResult initializationResult = initializationCoordinator.initialize(
                preferencesService,
                localizationService,
                turretVisionCoordinator,
                telemetry,
                "UNKNOWN",
                AutoConfig.HUMAN_PLAYER_RED_X,
                AutoConfig.HUMAN_PLAYER_RED_Y,
                AutoConfig.HUMAN_PLAYER_BLUE_X,
                AutoConfig.HUMAN_PLAYER_BLUE_Y,
                DriveConfig.ROBOT_CENTER_OFFSET_X,
                DriveConfig.ROBOT_CENTER_OFFSET_Y
        );
        String autoAlliance = initializationResult.autoAlliance;

        waitForStart();
        if (isStopRequested()) return;
        // Start follower in TeleOp drive mode before entering loop.
        follower.startTeleopDrive(true);
        follower.update();

        TeleOpLoopCoordinator.LoopState loopState = createInitialLoopState(autoAlliance);
        TeleOpLoopCoordinator.LoopDependencies loopDependencies = createLoopDependencies();

        while (this.opModeIsActive() && !isStopRequested()) {

            // Snapshot gamepad inputs once per loop so mapping stays centralized and traceable.
            DriverOneBindings driverOne = inputMapper.mapDriverOne(gamepad1);
            DriverTwoBindings driverTwo = inputMapper.mapDriverTwo(gamepad2);

            double currentTime = getRuntime();
            TeleOpLoopCoordinator.IterationResult iterationResult = loopCoordinator.runLoopIteration(
                    loopState,
                    driverOne,
                    driverTwo,
                    currentTime,
                    loopDependencies,
                    ejectionMotor.getVelocity() * 60 / DriveConfig.TICKS_PER_ROTATION,
                    ejectionMotor.getPower(),
                    ejectionMotor.getVelocity()
            );

            loopState = iterationResult.state;
            shootingPowerMode = loopState.shootingPowerMode;

            TeleOpStatusCoordinator.TraceState traceState = iterationResult.traceState;
            addTraceTelemetry("TeleOp", traceState.state, traceState.stateTimerSec);

            telemetry.update();
        } //while opModeIsActive

        stopRobot();
    } //runOpMode

    private TeleOpLoopCoordinator.LoopState createInitialLoopState(String autoAlliance) {
        return new TeleOpLoopCoordinator.LoopState(
                false,
                0,
                autoAlliance,
                shootingPowerMode,
                ShotgunPowerLevel.OFF
        );
    }

    private TeleOpLoopCoordinator.LoopDependencies createLoopDependencies() {
        return new TeleOpLoopCoordinator.LoopDependencies(
                autoParkCoordinator,
                driveControlCoordinator,
                intakeCoordinator,
                shootingCoordinator,
                odometryResetCoordinator,
                shooterPowerCoordinator,
                turretVisionCoordinator,
                turretModeCoordinator,
                turretCoordinator,
                statusCoordinator,
                telemetryCoordinator,
                localizationService,
                follower,
                intakeFSM,
                shotgunFSM,
                shootingFSM,
                turretFSM,
                gateFSM,
                telemetry,
                AutoConfig.AUTO_PARK_STICK_DEADZONE,
                AutoConfig.AUTO_PARK_TIMEOUT,
                DriveConfig.DRIVE_DEADZONE,
                DriveConfig.INPUT_EXPONENT,
                DriveConfig.SPEED_SCALE,
                DriveConfig.SPEED_SCALE_TURN,
                DriveConfig.ROTATION_SCALE,
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
                ShooterConfig.SHOOTING_POWER_ODOMETRY_Y_THRESHOLD,
                ShooterConfig.SHOOT_POWER_SELECT_STICK_THRESHOLD,
                ShooterConfig.SHOT_GUN_POWER_UP_RPM,
                ShooterConfig.SHOT_GUN_POWER_UP_FAR_RPM_TELEOP
        );
    }


} //TeleOpFSM class
