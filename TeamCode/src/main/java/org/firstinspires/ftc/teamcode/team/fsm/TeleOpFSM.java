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

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import android.content.SharedPreferences;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

@TeleOp(name = "TeleopFSM", group = "DriverControl")
@Config
@Configurable
public class TeleOpFSM extends DarienOpModeFSM {

    // INSTANCES
    private TelemetryManager panelsTelemetry;   // Panels Telemetry instance
    // follower is inherited from DarienOpModeFSM
    private GoBildaPinpointDriver odo;          // Pinpoint odometry driver for position reset

    // TUNING CONSTANTS
    public static double SHOT_TIMEOUT = 2.0; // seconds
    public static double ROTATION_SCALE = 0.4;
    public static double SPEED_SCALE = 0.5;

    // VARIABLES
    private double shotStartTime;
    //private boolean shotStarted = false;
    private boolean isReadingAprilTag = false;

    private ShotgunPowerLevel shotgunPowerLatch = ShotgunPowerLevel.OFF;


    // Turret fallback tracking
    private double lastCameraDetectionTime = 0;  // Timestamp of last successful camera detection

    // AUTOMATIC TURRET CONTROLS BASED ON CAMERA APRILTAG DETECTION
    AprilTagDetection detection;
    double yaw, range; // Stores detection.ftcPose.yaw
    double currentHeadingDeg;
    double relativeHeadingDeg; // Camera-relative bearing to AprilTag (degrees)
    double targetServoPos = Double.NaN; // Convert heading → servo position
    double rawBearingDeg; // Stores detection.ftcPose.bearing;

    double robotX, robotY, robotHeadingRadians;

    // cameraOffsetX < 0 if camera is mounted on the LEFT
// public static double cameraOffsetX = 0.105; // in centimeter, positive is right, negative is left
    double correctedBearingRad;
    double correctedBearingDeg;
    boolean isCalculatingTurretTargetPosition = false;

    int targetGoalTagId;
    private String autoAlliance = "UNKNOWN";

    @Override
    public void initControls() {
        super.initControls();
        gateFSM.close();
        turretFSM.center(); // set to center position

        // Initialize GoBildaPinpointDriver for odometry position reset
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    }


    @Override
    public void runOpMode() throws InterruptedException {
        float gain = 2;
        initControls();
        tp = new TelemetryPacket();
        dash = FtcDashboard.getInstance();

        SharedPreferences prefs = AppUtil.getInstance().getActivity().getSharedPreferences("ftc_prefs", android.content.Context.MODE_PRIVATE);
        autoAlliance = prefs.getString("auto_alliance", "UNKNOWN");

        // Set align color based on saved color from auto
        if ("BLUE".equals(autoAlliance)) {
            targetGoalTagId = APRILTAG_ID_GOAL_BLUE;
            turretFSM.setOffsetBlue();
        } else if ("RED".equals(autoAlliance)) {
            targetGoalTagId = APRILTAG_ID_GOAL_RED;
            turretFSM.setOffsetRed();
        }

        // Load saved odometry position from auto (if available)
        boolean hasAutoPosition = prefs.contains("auto_final_x");
        if (hasAutoPosition) {
            double autoX = prefs.getFloat("auto_final_x", 0f);
            double autoY = prefs.getFloat("auto_final_y", 0f);
            double autoHeadingRad = prefs.getFloat("auto_final_heading", 0f);

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
            follower.setTeleOpDrive(-gamepad1.left_stick_y * SPEED_SCALE, -gamepad1.left_stick_x * SPEED_SCALE, -gamepad1.right_stick_x * ROTATION_SCALE, true);
            follower.update();
            gateFSM.update(getRuntime(), true, telemetry);
            turretFSM.update();

            // INTAKE FSM UPDATE — runs sensor polling and auto-stops when full
            if (intakeFSM.getState() == IntakeFSM.States.INTAKING) {
                intakeFSM.updateIntaking(getRuntime(), true, telemetry);
            }

            // SHOOTING FSM UPDATE — drives spin-up → gate open → gate close → done
            if (shootingFSM.getStage() != ShootingFSM.Stage.IDLE) {
                shootingFSM.update(getRuntime(), telemetry);
                if (shootingFSM.isDone()) {
                    shootingFSM.reset();
                    intakeFSM.startIntaking();
                }
            }

            // CAMERA-BASED TURRET CONTROL
            if (turretFSM.getState() == TurretFSM.TurretStates.CAMERA) {
                if (!isReadingAprilTag) {
                    startReadingGoalId();
                } else {
                    updateReadingGoalId();
                }
            }

            // -----------------
            // GAMEPAD1 CONTROLS
            // -----------------

            if (gamepad1.y || gamepad1.right_bumper) {
                // Intake on
                intakeFSM.startIntaking();
            } else if (gamepad1.a) {
                // Eject mode
                intakeFSM.reverse();
            } else if (gamepad1.x) {
                // Intake "Off"
                intakeFSM.off();
            }

            if (gamepad2.left_bumper) {
                gateFSM.close();
            } else if (gamepad2.rightBumperWasPressed()) {
                // Start shoot sequence — FAR power if right stick pushed up, CLOSE power otherwise
                ShootingFSM.PowerLevel power = (gamepad2.right_stick_y < -0.05)
                        ? ShootingFSM.PowerLevel.FAR
                        : ShootingFSM.PowerLevel.CLOSE;
                shootingFSM.start(getRuntime(), power);
            } else if (gamepad2.rightBumperWasReleased()) {
                shootingFSM.finish();
            }

            // Add debug telemetry
            telemetry.addData("GATE: State", gateFSM.getState().toString());
            telemetry.addData("INTAKE: State", intakeFSM.getState().toString());
            telemetry.addData("SHOOTING: Stage", shootingFSM.getStage().toString());
            //telemetry.addData("rubberBandsFront Power", rubberBandsFront.getPower());
            //telemetry.addData("rubberBandsMid Power", rubberBandsMid.getPower());


            // ODOMETRY RESET BUTTON - Reset to human player starting position
            if (gamepad1.dpadUpWasPressed()) {
                // Determine which human player position based on alliance color
                double resetX = 0, resetY = 0, resetHdeg = 0;
                if ("RED".equals(autoAlliance)) {
                    resetX = HUMAN_PLAYER_RED_X + ROBOT_CENTER_OFFSET_X;
                    resetY = HUMAN_PLAYER_RED_Y + ROBOT_CENTER_OFFSET_Y;
                    resetHdeg = 180; // Front-first into red corner: robot drives straight into red wall (-X direction)
                    telemetry.addLine("ODOMETRY RESET: Red Human Player Position (0, 0)");
                } else if ("BLUE".equals(autoAlliance)) {
                    resetX = HUMAN_PLAYER_BLUE_X - ROBOT_CENTER_OFFSET_X;
                    resetY = HUMAN_PLAYER_BLUE_Y + ROBOT_CENTER_OFFSET_Y;
                    resetHdeg = 0; // Front-first into blue corner: robot drives straight into blue wall (+X direction)
                    telemetry.addLine("ODOMETRY RESET: Blue Human Player Position (144, 0)");
                }
            /*
            else {
                // Default to (0,0) if alliance unknown
                resetX = 0;
                resetY = 0;
                telemetry.addLine("ODOMETRY RESET: Default Position (0, 0)");
            }
            */

                // Reset the pinpoint odometry position
                odo.setPosition(new Pose2D(
                        DistanceUnit.INCH,
                        resetX,
                        resetY,
                        AngleUnit.DEGREES,
                        resetHdeg
                ));

                // Update the follower's pose to match
                follower.setPose(new Pose(resetX, resetY, Math.toRadians(resetHdeg)));

                telemetry.addData("New Odometry Position", String.format("(%.1f, %.1f, %.1f)", resetX, resetY, resetHdeg));
            }

            // -----------------
            // GAMEPAD2 CONTROLS
            // -----------------

            //SET ALLIANCE COLOR CONTROL
            if (gamepad2.b && !isReadingAprilTag) {
                // ALIGN TO RED GOAL
                autoAlliance = "RED";
                targetGoalTagId = APRILTAG_ID_GOAL_RED;
                turretFSM.setOffsetRed();
                telemetry.addLine("ALLIANCE SET TO RED!");
            } else if (gamepad2.x && !isReadingAprilTag) {
                // ALIGN TO BLUE GOAL
                autoAlliance = "BLUE";
                targetGoalTagId = APRILTAG_ID_GOAL_BLUE;
                turretFSM.setOffsetBlue();
                telemetry.addLine("ALLIANCE SET TO BLUE!");
            }

            //TURRET STATE CHANGE CONTROLS
            if (gamepad2.dpadUpWasPressed()) {
                turretFSM.setState(TurretFSM.TurretStates.ODOMETRY);
                shootingPowerMode = ShootingPowerModes.ODOMETRY;
            } else if (gamepad2.dpadDownWasPressed()) {
                turretFSM.setState(TurretFSM.TurretStates.CAMERA);
                startReadingGoalId();
            }

            /* NOT BEING USED
            // -----------------
            // SHOOTING MACRO (dpad_down) — start a timed single-shot sequence
            // -----------------
            if (gamepad2.dpad_down && !shotStarted) {
                ShootingFSM.PowerLevel power = (gamepad2.right_stick_y < -0.05)
                        ? ShootingFSM.PowerLevel.FAR
                        : ShootingFSM.PowerLevel.CLOSE;
                shootingFSM.start(getRuntime(), power);
                shotStartTime = getRuntime();
                shotStarted = true;
            }
            if (shotStarted) {
                if (shootingFSM.isDone() || getRuntime() - shotStartTime >= SHOT_TIMEOUT) {
                    shootingFSM.reset();
                    shotStarted = false;
                }
            }
             */


            // Get current robot pose from follower
            robotX = follower.getPose().getX();
            robotY = follower.getPose().getY();
            robotHeadingRadians = follower.getPose().getHeading();

            // TURRET CONTROLS
            // Stick direction is checked first; trigger modulates speed within that direction.
            if (gamepad2.left_stick_x <= -0.05) {
                if (gamepad2.left_trigger > 0.25) {
                    turretFSM.rotateLeftFast();
                } else {
                    turretFSM.rotateLeft();
                }
                turretFSM.setState(TurretFSM.TurretStates.MANUAL);
            } else if (gamepad2.left_stick_x >= 0.05) {
                if (gamepad2.left_trigger > 0.25) {
                    turretFSM.rotateRightFast();
                } else {
                    turretFSM.rotateRight();
                }
                turretFSM.setState(TurretFSM.TurretStates.MANUAL);
            } else if (gamepad2.left_stick_button) {
                turretFSM.center();
                turretFSM.setState(TurretFSM.TurretStates.MANUAL);
            }
            // Odometry-based turret aiming (when not in manual control)
            else if (turretFSM.getState() == TurretFSM.TurretStates.ODOMETRY) {

                // Determine target goal based on alliance color
                double targetGoalX, targetGoalY;
                if ("RED".equals(autoAlliance)) {
                    targetGoalX = DarienOpModeFSM.GOAL_RED_X;
                    targetGoalY = DarienOpModeFSM.GOAL_RED_Y;
                } else {
                    targetGoalX = DarienOpModeFSM.GOAL_BLUE_X;
                    targetGoalY = DarienOpModeFSM.GOAL_BLUE_Y;
                }

                turretFSM.setPositionFromOdometry(targetGoalX, targetGoalY, robotX, robotY, robotHeadingRadians);
            }

            //CONTROL: EJECTION MOTORS
            //ODOMETRY BASED SHOOT POWER
            if (shootingPowerMode == ShootingPowerModes.ODOMETRY) {
                // Automatic power selection based on robot Y position
                if (robotY <= SHOOTING_POWER_ODOMETRY_Y_THRESHOLD) {
                    shotgunPowerLatch = ShotgunPowerLevel.HIGH;
                } else {
                    shotgunPowerLatch = ShotgunPowerLevel.LOW;
                }
            }

            //Latch control - manual override switches back to MANUAL mode
            if (gamepad2.right_stick_y < -.05) {
                shotgunPowerLatch = ShotgunPowerLevel.HIGH;
                shootingPowerMode = ShootingPowerModes.MANUAL;
            } else if (gamepad2.right_stick_y > 0.05) {
                shotgunPowerLatch = ShotgunPowerLevel.LOW;
                shootingPowerMode = ShootingPowerModes.MANUAL;
            } else if (gamepad2.rightStickButtonWasPressed() || gamepad2.a) {
                shotgunPowerLatch = ShotgunPowerLevel.OFF;
                shootingPowerMode = ShootingPowerModes.MANUAL;
            }
            switch (shotgunPowerLatch) {
                case OFF:
                    shotgunFSM.toOff();
                    telemetry.addData("Requested ShotGun RPM", 0);
                    break;
                case HIGH:
                    shotgunFSM.toPowerUpFar(SHOT_GUN_POWER_UP_FAR_RPM_TELEOP);
                    telemetry.addData("Requested ShotGun RPM", SHOT_GUN_POWER_UP_FAR_RPM_TELEOP);
                    break;
                case LOW:
                default:
                    shotgunFSM.toPowerUp(SHOT_GUN_POWER_UP_RPM);
                    telemetry.addData("Requested ShotGun RPM", SHOT_GUN_POWER_UP_RPM);
                    break;
            }
            telemetry.addData("Actual ShotGun RPM", ejectionMotor.getVelocity() * 60 / TICKS_PER_ROTATION); // convert from ticks per second to RPM
            telemetry.addData("ejectionMotor power", ejectionMotor.getPower());
            telemetry.addData("Actual ShotGun TPS", ejectionMotor.getVelocity()); // convert from ticks per second to RPM
            telemetry.addData("Shooting Power Mode", shootingPowerMode.toString());
            telemetry.addData("Shotgun Power Latch", shotgunPowerLatch.toString());

            // Display alliance color from SharedPreferences
            telemetry.addData("Alliance Color from Auto", autoAlliance);
            telemetry.addData("Target AprilTag ID", targetGoalTagId);
            // telemetry.addData("Time Since Last Camera Detection (ms)",
            //       (getRuntime() - lastCameraDetectionTime) * 1000);
            telemetry.addData("Odometry Pos (X,Y)", String.format("%.1f, %.1f", robotX, robotY));
            telemetry.addData("Odometry Bearing (deg)", String.format("%.1f", Math.toDegrees(robotHeadingRadians)));
            telemetry.addData("TURRET: State", turretFSM.getState().toString());
            telemetry.addData("TURRET: Current Turret Pos", turretFSM.getPosition());

            telemetry.update();
        } //while opModeIsActive
    } //runOpMode

    private void startReadingGoalId() {
        tagFSM.start(getRuntime());
        isReadingAprilTag = true;
    }

    private void updateReadingGoalId() {
        tagFSM.update(getRuntime(), true, telemetry);
        telemetry.addLine("Goal Detection: Reading...");

        if (tagFSM.isDone()) {
            telemetry.addLine("Goal Detection: DONE reading!");
            isReadingAprilTag = false;
            aprilTagDetections = tagFSM.getDetections();
            //aprilTagDetections.removeIf(tag -> tag.id != 24);
            if (targetGoalTagId == APRILTAG_ID_GOAL_RED) {
                aprilTagDetections.removeIf(tag -> tag.id == 20 || tag.id == 21 || tag.id == 22 || tag.id == 23);
                turretFSM.setOffsetRed();
            } else if (targetGoalTagId == APRILTAG_ID_GOAL_BLUE) {
                aprilTagDetections.removeIf(tag -> tag.id == 24 || tag.id == 21 || tag.id == 22 || tag.id == 23);
                turretFSM.setOffsetBlue();
            }
            if (!aprilTagDetections.isEmpty()) {
                telemetry.addLine("Goal Detection: FOUND APRILTAG!");
                // Rotate the turret only if an apriltag is detected and it's the target goal apriltag id
                detection = aprilTagDetections.get(0);
                if (detection.id == targetGoalTagId && detection.ftcPose != null) {
                    telemetry.addLine("Goal Detection: ALIGNING TURRET TO GOAL " + targetGoalTagId);
                    //yaw = detection.ftcPose.yaw; //  REMOVE LATER SINCE IT'S ONLY FOR TELEMETRY
                    //range = detection.ftcPose.range; //  REMOVE LATER SINCE IT'S ONLY FOR TELEMETRY

                    // Current turret heading (degrees)
                    //currentHeadingDeg = turretFSM.getTurretHeading(); //  REMOVE LATER SINCE IT'S ONLY FOR TELEMETRY

                    // Camera-relative bearing to AprilTag (degrees)
                    rawBearingDeg = detection.ftcPose.bearing;

                    if (!isCalculatingTurretTargetPosition) {
                        isCalculatingTurretTargetPosition = true;
                        turretFSM.alignToBearing(rawBearingDeg);
                        // lastCameraDetectionTime = getRuntime();  // Update last detection time
                    }
                } else if (detection.ftcPose == null) {
                    telemetry.addLine("Goal Detection: WARNING - Pose estimation failed!");
                } // end detection.id == 20 or 24
            } // end detection is empty
        } // end tagFSM is done
        isCalculatingTurretTargetPosition = false;
    }

} //TeleOpFSM class
