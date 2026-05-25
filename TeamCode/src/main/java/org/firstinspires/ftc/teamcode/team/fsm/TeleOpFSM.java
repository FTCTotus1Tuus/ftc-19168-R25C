package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;

import android.annotation.SuppressLint;
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

@TeleOp(name = "TeleopFSM", group = "DriverControl")
@Config
@Configurable
public class TeleOpFSM extends DarienOpModeFSM {

    // INSTANCES
    // follower is inherited from DarienOpModeFSM
    private GoBildaPinpointDriver odo;          // Pinpoint odometry driver for position reset

    // TUNING CONSTANTS
    public static double ROTATION_SCALE = 0.5;
    public static double SPEED_SCALE = 1.0;
    public static double SPEED_SCALE_TURN = 0.8;
    public static double INPUT_EXPONENT = 3.0; // 1.0=linear, 2.0=squared, 3.0=cubed (preserves sign)
    public static double SHOOT_POWER_SELECT_STICK_THRESHOLD = 0.05;

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
    private static final double AUTO_PARK_STICK_DEADZONE = 0.1; // stick threshold to cancel auto-park
    public static double DEADZONE = 0.1;


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
        turretVisionCoordinator = new TurretVisionCoordinator(tagFSM, turretFSM, APRILTAG_ID_GOAL_BLUE, APRILTAG_ID_GOAL_RED);

        // Initialize GoBildaPinpointDriver for odometry position reset
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
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

        // Load saved odometry position from auto (if available)
        boolean hasAutoPosition = preferencesService.hasAutoFinalPose();
        if (hasAutoPosition) {
            double autoX = preferencesService.getAutoFinalX(0f);
            double autoY = preferencesService.getAutoFinalY(0f);
            double autoHeadingRad = preferencesService.getAutoFinalHeading(0f);

            // Set odometry position from auto
            odo.setPosition(new Pose2D(
                    DistanceUnit.INCH,
                    autoX,
                    autoY,
                    AngleUnit.RADIANS,
                    autoHeadingRad
            ));

            // Update follower pose to match
            follower.setPose(new Pose(autoX, autoY, autoHeadingRad));

            telemetry.addLine("=== ODOMETRY LOADED FROM AUTO ===");
            telemetry.addData("Loaded Position", String.format("X=%.1f, Y=%.1f, H=%.1f°",
                                                               autoX, autoY, Math.toDegrees(autoHeadingRad)));
        } else {
            // No auto position saved - default to human player position based on alliance
            double resetX = 0, resetY = 0, resetHdeg = 0;
            if ("RED".equals(autoAlliance)) {
                resetX = HUMAN_PLAYER_RED_X + ROBOT_CENTER_OFFSET_X;
                resetY = HUMAN_PLAYER_RED_Y + ROBOT_CENTER_OFFSET_Y;
                resetHdeg = 180; // Front-first into red corner
            } else if ("BLUE".equals(autoAlliance)) {
                resetX = HUMAN_PLAYER_BLUE_X - ROBOT_CENTER_OFFSET_X;
                resetY = HUMAN_PLAYER_BLUE_Y + ROBOT_CENTER_OFFSET_Y;
                resetHdeg = 0; // Front-first into blue corner
            }

            odo.setPosition(new Pose2D(
                    DistanceUnit.INCH,
                    resetX,
                    resetY,
                    AngleUnit.DEGREES,
                    resetHdeg
            ));

            follower.setPose(new Pose(resetX, resetY, Math.toRadians(resetHdeg)));

            telemetry.addLine("=== NO AUTO DATA - DEFAULT POSITION ===");
            telemetry.addData("Default Position", String.format("X=%.1f, Y=%.1f, H=%.1f°",
                                                                resetX, resetY, resetHdeg));
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
                    AUTO_PARK_STICK_DEADZONE,
                    AUTO_PARK_TIMEOUT,
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
                    DEADZONE,
                    INPUT_EXPONENT,
                    SPEED_SCALE,
                    SPEED_SCALE_TURN,
                    ROTATION_SCALE,
                    follower
            );

            follower.update();

            gateFSM.update(getRuntime(), telemetry);
            turretFSM.update(getRuntime(), telemetry);

            // INTAKE FSM UPDATE — runs sensor polling and auto-stops when full
            intakeCoordinator.updateActiveIntake(getRuntime(), telemetry);

            // SHOOTING FSM UPDATE — drives spin-up → gate open → gate close → done
            shootingCoordinator.updateActiveSequence(getRuntime(), telemetry);

            // CAMERA-BASED TURRET CONTROL
            turretVisionCoordinator.updateCameraControl(getRuntime(), telemetry);

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
                    PARK_RED_X,
                    PARK_RED_Y,
                    PARK_RED_H_DEG,
                    PARK_BLUE_X,
                    PARK_BLUE_Y,
                    PARK_BLUE_H_DEG,
                    AUTO_PARK_POWER,
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
                    SHOOT_POWER_SELECT_STICK_THRESHOLD
            );

            odometryResetCoordinator.tryResetToHumanPlayerPosition(
                    gamepad1.dpadUpWasPressed(),
                    autoAlliance,
                    HUMAN_PLAYER_RED_X,
                    HUMAN_PLAYER_RED_Y,
                    HUMAN_PLAYER_BLUE_X,
                    HUMAN_PLAYER_BLUE_Y,
                    ROBOT_CENTER_OFFSET_X,
                    ROBOT_CENTER_OFFSET_Y,
                    odo,
                    follower,
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



            // Snapshot pose once per loop so all coordinators/telemetry use the same frame.
            double robotX = follower.getPose().getX();
            double robotY = follower.getPose().getY();
            double robotHeadingRadians = follower.getPose().getHeading();

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
                    SHOOTING_POWER_ODOMETRY_Y_THRESHOLD,
                    gamepad2.right_stick_y,
                    SHOOT_POWER_SELECT_STICK_THRESHOLD,
                    gamepad2.rightStickButtonWasPressed(),
                    gamepad2.a
            );
            shootingPowerMode = powerState.mode;
            shotgunPowerLatch = powerState.latch;

            shooterPowerCoordinator.applyRequestedPower(
                    shotgunFSM,
                    shotgunPowerLatch,
                    SHOT_GUN_POWER_UP_RPM,
                    SHOT_GUN_POWER_UP_FAR_RPM_TELEOP,
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
                    ejectionMotor.getVelocity() * 60 / TICKS_PER_ROTATION,
                    ejectionMotor.getPower(),
                    ejectionMotor.getVelocity(),
                    autoAlliance,
                    turretVisionCoordinator.getTargetGoalTagId(),
                    robotX,
                    robotY,
                    robotHeadingRadians,
                    isAutoParking,
                    PARK_RED_X,
                    PARK_RED_Y,
                    PARK_BLUE_X,
                    PARK_BLUE_Y,
                    AUTO_PARK_TIMEOUT,
                    autoParkStartTime,
                    getRuntime()
            );

            String traceState = isAutoParking ? "AUTO_PARK" : "DRIVER_CONTROL";
            double traceStateTimer = isAutoParking ? (getRuntime() - autoParkStartTime) : 0.0;
            addTraceTelemetry("TeleOp", traceState, traceStateTimer);

            telemetry.update();
        } //while opModeIsActive
    } //runOpMode


} //TeleOpFSM class
