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
import org.firstinspires.ftc.teamcode.team.core.DriverOneBindings;
import org.firstinspires.ftc.teamcode.team.core.DriverTwoBindings;
import org.firstinspires.ftc.teamcode.team.core.TeleOpIterationResult;
import org.firstinspires.ftc.teamcode.team.core.TeleOpCoordinatorSet;
import org.firstinspires.ftc.teamcode.team.core.TeleOpInitializationCoordinator;
import org.firstinspires.ftc.teamcode.team.core.TeleOpLoopDependencies;
import org.firstinspires.ftc.teamcode.team.core.TeleOpLoopState;
import org.firstinspires.ftc.teamcode.team.core.TeleOpStatusCoordinator;

@TeleOp(name = "TeleopFSM", group = "DriverControl")
@Config
@Configurable
public class TeleOpFSM extends DarienOpModeFSM {

    private TeleOpCoordinatorSet coordinators;

    @Override
    public void initControls() {
        super.initControls();
        gateFSM.close();
        turretFSM.center(); // set to center position
        coordinators = TeleOpCoordinatorSet.create(
                aprilTagService,
                turretFSM,
                intakeFSM,
                shootingFSM,
                gateFSM
        );
    }


    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        initControls();
        tp = new TelemetryPacket();
        dash = FtcDashboard.getInstance();

        TeleOpInitializationCoordinator.InitializationResult initializationResult = coordinators.initializationCoordinator.initialize(
                preferencesService,
                localizationService,
                coordinators.turretVisionCoordinator,
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

        TeleOpLoopState loopState = TeleOpLoopState.initial(autoAlliance, shootingPowerMode);
        TeleOpLoopDependencies loopDependencies = TeleOpLoopDependencies.create(
                coordinators,
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

        while (this.opModeIsActive() && !isStopRequested()) {

            // Snapshot gamepad inputs once per loop so mapping stays centralized and traceable.
            DriverOneBindings driverOne = coordinators.inputMapper.mapDriverOne(gamepad1);
            DriverTwoBindings driverTwo = coordinators.inputMapper.mapDriverTwo(gamepad2);

            double currentTime = getRuntime();
            TeleOpIterationResult iterationResult = coordinators.loopCoordinator.runLoopIteration(
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

} //TeleOpFSM class
