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
import org.firstinspires.ftc.teamcode.team.core.IntakeCoordinator;
import org.firstinspires.ftc.teamcode.team.core.OdometryResetCoordinator;
import org.firstinspires.ftc.teamcode.team.core.ShooterPowerCoordinator;
import org.firstinspires.ftc.teamcode.team.core.ShootingCoordinator;
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

            // -----------------
            // ALWAYS RUN
            // -----------------

            AutoParkCoordinator.AutoParkResult progressResult = autoParkCoordinator.updateAutoParkProgress(
                    isAutoParking,
                    autoParkStartTime,
                    getRuntime(),
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    AutoConfig.AUTO_PARK_STICK_DEADZONE,
                    AutoConfig.AUTO_PARK_TIMEOUT,
                    follower,
                    telemetry,
                    shotgunPowerLatch
            );
            isAutoParking = progressResult.isAutoParking;
            autoParkStartTime = progressResult.autoParkStartTime;
            shotgunPowerLatch = progressResult.shotgunPowerLatch;

            // Driver stick shaping + drive command emission are centralized in this coordinator.
            driveControlCoordinator.applyTeleOpDrive(
                    isAutoParking,
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    DriveConfig.DRIVE_DEADZONE,
                    DriveConfig.INPUT_EXPONENT,
                    DriveConfig.SPEED_SCALE,
                    DriveConfig.SPEED_SCALE_TURN,
                    DriveConfig.ROTATION_SCALE,
                    follower
            );

            follower.update();

            gateFSM.update(getRuntime(), telemetry);
            turretFSM.update(getRuntime(), telemetry);

            // INTAKE FSM UPDATE — runs sensor polling and auto-stops when full
            intakeCoordinator.updateActiveIntake(getRuntime(), telemetry);

            // SHOOTING FSM UPDATE — drives spin-up → gate open → gate close → done
            shootingCoordinator.updateActiveSequence(getRuntime(), telemetry);

            // Snapshot pose once per loop so all coordinators/telemetry use the same frame.
            double robotX = follower.getPose().getX();
            double robotY = follower.getPose().getY();
            double robotHeadingRadians = follower.getPose().getHeading();

            // Camera control runs before manual/odometry controls so driver manual intent can still override.
            turretVisionCoordinator.updateCameraControl(
                    getRuntime(),
                    autoAlliance,
                    robotX,
                    robotY,
                    robotHeadingRadians,
                    telemetry
            );

            // -----------------
            // GAMEPAD1 CONTROLS
            // -----------------

            intakeCoordinator.handleDriverControls(
                    gamepad1.y || gamepad1.right_bumper,
                    gamepad1.a,
                    gamepad1.x
            );

            // Auto-park start logic returns the next loop state in one place.
            AutoParkCoordinator.AutoParkResult startResult = autoParkCoordinator.tryStartAutoPark(
                    gamepad1.bWasPressed(),
                    isAutoParking,
                    getRuntime(),
                    autoParkStartTime,
                    autoAlliance,
                    AutoConfig.PARK_RED_X,
                    AutoConfig.PARK_RED_Y,
                    AutoConfig.PARK_RED_H_DEG,
                    AutoConfig.PARK_BLUE_X,
                    AutoConfig.PARK_BLUE_Y,
                    AutoConfig.PARK_BLUE_H_DEG,
                    AutoConfig.AUTO_PARK_POWER,
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
                    getRuntime(),
                    gamepad2.left_bumper,
                    gamepad2.rightBumperWasPressed(),
                    gamepad2.rightBumperWasReleased(),
                    gamepad2.right_stick_y,
                    ShooterConfig.SHOOT_POWER_SELECT_STICK_THRESHOLD
            );

            odometryResetCoordinator.tryResetToHumanPlayerPosition(
                    gamepad1.dpadUpWasPressed(),
                    autoAlliance,
                    AutoConfig.HUMAN_PLAYER_RED_X,
                    AutoConfig.HUMAN_PLAYER_RED_Y,
                    AutoConfig.HUMAN_PLAYER_BLUE_X,
                    AutoConfig.HUMAN_PLAYER_BLUE_Y,
                    DriveConfig.ROBOT_CENTER_OFFSET_X,
                    DriveConfig.ROBOT_CENTER_OFFSET_Y,
                    localizationService,
                    telemetry
            );

            // -----------------
            // GAMEPAD2 CONTROLS
            // -----------------

            //SET ALLIANCE COLOR CONTROL
            TurretVisionCoordinator.AllianceSwitchResult allianceSwitchResult = turretVisionCoordinator.handleAllianceButtons(
                    gamepad2.b,
                    gamepad2.x,
                    autoAlliance,
                    telemetry
            );
            autoAlliance = allianceSwitchResult.alliance;

            TurretModeCoordinator.ModeSwitchResult modeSwitchResult = turretModeCoordinator.handleModeSwitches(
                    gamepad2.dpadUpWasPressed(),
                    gamepad2.dpadDownWasPressed(),
                    shootingPowerMode
            );
            shootingPowerMode = modeSwitchResult.shootingPowerMode;
            if (modeSwitchResult.shouldStartGoalReading) {
                turretVisionCoordinator.startReadingGoalId(getRuntime());
            }


            // Turret coordinator resolves manual stick intent first, then odometry aiming fallback.
            turretCoordinator.applyManualOrOdometryControl(
                    autoAlliance,
                    gamepad2.left_stick_x,
                    gamepad2.left_trigger,
                    gamepad2.left_stick_button,
                    robotX,
                    robotY,
                    robotHeadingRadians
            );

            // Compute latch/mode first, then apply shooter power command.
            ShooterPowerCoordinator.PowerState powerState = shooterPowerCoordinator.computePowerState(
                    shootingPowerMode,
                    shotgunPowerLatch,
                    robotY,
                    ShooterConfig.SHOOTING_POWER_ODOMETRY_Y_THRESHOLD,
                    gamepad2.right_stick_y,
                    ShooterConfig.SHOOT_POWER_SELECT_STICK_THRESHOLD,
                    gamepad2.rightStickButtonWasPressed(),
                    gamepad2.a
            );
            shootingPowerMode = powerState.mode;
            shotgunPowerLatch = powerState.latch;

            shooterPowerCoordinator.applyRequestedPower(
                    shotgunFSM,
                    shotgunPowerLatch,
                    ShooterConfig.SHOT_GUN_POWER_UP_RPM,
                    ShooterConfig.SHOT_GUN_POWER_UP_FAR_RPM_TELEOP,
                    telemetry
            );
            telemetryCoordinator.addLoopTelemetry(
                    telemetry,
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
                    turretVisionCoordinator.getTargetGoalTagId(),
                    robotX,
                    robotY,
                    robotHeadingRadians,
                    isAutoParking,
                    AutoConfig.PARK_RED_X,
                    AutoConfig.PARK_RED_Y,
                    AutoConfig.PARK_BLUE_X,
                    AutoConfig.PARK_BLUE_Y,
                    AutoConfig.AUTO_PARK_TIMEOUT,
                    autoParkStartTime,
                    getRuntime()
            );

            telemetry.addData("Vision Status", turretVisionCoordinator.getVisionStatusLine());
            telemetry.addData("Vision Fallback", turretVisionCoordinator.getFallbackStatusLine());
            telemetry.addData("Vision Age (ms)", String.format("%.0f", turretVisionCoordinator.getLastCameraAgeMs()));

            String traceState = isAutoParking ? "AUTO_PARK" : "DRIVER_CONTROL";
            double traceStateTimer = isAutoParking ? (getRuntime() - autoParkStartTime) : 0.0;
            addTraceTelemetry("TeleOp", traceState, traceStateTimer);

            telemetry.update();
        } //while opModeIsActive

        stopRobot();
    } //runOpMode


} //TeleOpFSM class
