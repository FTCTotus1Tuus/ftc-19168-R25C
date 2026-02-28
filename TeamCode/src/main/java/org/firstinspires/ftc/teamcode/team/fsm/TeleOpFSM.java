package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
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
    public Follower follower;                   // Pedro Pathing follower instance
    private GoBildaPinpointDriver odo;          // Pinpoint odometry driver for position reset

    // TUNING CONSTANTS
    public static double INTAKE_TIME = 1;
    public static double SHOT_TIMEOUT = 2.0; // seconds
    public static double ROTATION_SCALE = 0.4;

    // VARIABLES
    private double shotStartTime;
    private boolean shotStarted = false;
    private boolean isReadingAprilTag = false;
    private boolean hasIntakeSensorDetected = false;
    private boolean hasMiddleSensorDetected = false;
    private boolean hasTurretSensorDetected = false;

    // Color sensor detection timing
    private double intakeColorSensorDetectionStartTime = Double.NaN;
    private double middleColorSensorDetectionStartTime = Double.NaN;
    private double turretColorSensorDetectionStartTime = Double.NaN;
    private static final double DETECTION_HOLD_TIME = 2.0; // seconds

    private ShotgunPowerLevel shotgunPowerLatch = ShotgunPowerLevel.LOW;

    private enum ShootingPowerModes {MANUAL, ODOMETRY}

    private ShootingPowerModes shootingPowerMode = ShootingPowerModes.MANUAL;

    // Turret fallback tracking
    private double lastCameraDetectionTime = 0;  // Timestamp of last successful camera detection

    private enum IntakeModes {OFF, FORWARD, REVERSE, FULL, SHOOT}
    private IntakeModes intakeMode = IntakeModes.OFF;

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
        follower = Constants.createFollower(hardwareMap);
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
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * ROTATION_SCALE, true);
            follower.update();
            gateFSM.update(getRuntime(), true, telemetry);
            turretFSM.update();

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

            //RubberBands + topIntake CONTROLS:
            if (gamepad1.y) {
                // Intake on
                intakeMode = IntakeModes.FORWARD;
                gateFSM.close();
                // Reset detection flags and timers when starting intake
                hasIntakeSensorDetected = false;
                hasMiddleSensorDetected = false;
                hasTurretSensorDetected = false;
                intakeColorSensorDetectionStartTime = Double.NaN;
                middleColorSensorDetectionStartTime = Double.NaN;
                turretColorSensorDetectionStartTime = Double.NaN;
            } else if (gamepad1.a) {
                // Eject mode
                intakeMode = IntakeModes.REVERSE;
            } else if (gamepad1.x) {
                // Intake "Off"
                intakeMode = IntakeModes.OFF;
            }

            if (gamepad2.left_bumper) {
                gateFSM.close();
            } else if (gamepad2.right_bumper) {
                gateFSM.open();
                intakeMode = IntakeModes.SHOOT;
                hasIntakeSensorDetected = false;
                hasMiddleSensorDetected = false;
                hasTurretSensorDetected = false;
                intakeColorSensorDetectionStartTime = Double.NaN;
                middleColorSensorDetectionStartTime = Double.NaN;
                turretColorSensorDetectionStartTime = Double.NaN;
                setLedRed();
            }

            // Set motor powers based on current state (runs every loop)
            switch (intakeMode) {
                case SHOOT:
                    rubberBandsFront.setPower(-INTAKE_RUBBER_BANDS_POWER_HIGH);
                    rubberBandsMid.setPower(-INTAKE_INTAKE_ROLLER_POWER_HIGH);
                    rampServoLow.setPower(INTAKE_INTAKE_ROLLER_POWER_HIGH);
                    rampServoHigh.setPower(INTAKE_INTAKE_ROLLER_POWER_HIGH);
                    intakeRear.setPower(-INTAKE_INTAKE_ROLLER_POWER_HIGH);
                    break;
                case FORWARD:
                    rubberBandsFront.setPower(-INTAKE_RUBBER_BANDS_POWER);
                    rubberBandsMid.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                    rampServoLow.setPower(INTAKE_INTAKE_ROLLER_POWER);
                    rampServoHigh.setPower(INTAKE_INTAKE_ROLLER_POWER);
                    intakeRear.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                    gateFSM.close();
                    break;
                case REVERSE:
                    rubberBandsFront.setPower(OUTPUT_RUBBER_BANDS_POWER);
                    rubberBandsMid.setPower(OUTPUT_RUBBER_BANDS_POWER);
                    rampServoLow.setPower(-OUTPUT_RUBBER_BANDS_POWER);
                    rampServoHigh.setPower(-OUTPUT_RUBBER_BANDS_POWER);
                    intakeRear.setPower(OUTPUT_RUBBER_BANDS_POWER);
                    break;
                case OFF:
                    rubberBandsFront.setPower(0);
                    rubberBandsMid.setPower(0);
                    rampServoLow.setPower(0);
                    rampServoHigh.setPower(0);
                    intakeRear.setPower(0);
                    break;
                case FULL:
                default:
                    rubberBandsFront.setPower(0);
                    rubberBandsMid.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                    rampServoLow.setPower(INTAKE_INTAKE_ROLLER_POWER);
                    rampServoHigh.setPower(INTAKE_INTAKE_ROLLER_POWER);
                    intakeRear.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                    break;
            }


            // Add debug telemetry
            telemetry.addData("Gate", gateFSM.getState().toString());
            telemetry.addData("rubberBandsFront Power", rubberBandsFront.getPower());
            telemetry.addData("rubberBandsMid Power", rubberBandsMid.getPower());

            if (intakeColorSensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) intakeColorSensor).getDistance(DistanceUnit.CM));

                if (((DistanceSensor) intakeColorSensor).getDistance(DistanceUnit.CM) <= INTAKE_DISTANCE) {
                    if (Double.isNaN(intakeColorSensorDetectionStartTime)) {
                        intakeColorSensorDetectionStartTime = getRuntime();
                    }
                    if (getRuntime() - intakeColorSensorDetectionStartTime >= DETECTION_HOLD_TIME) {
                        telemetry.addData("intakeColorSensor", "Detected");
                        hasIntakeSensorDetected = true;
                    } else {
                        telemetry.addData("intakeColorSensor", "Detecting... %.1fs", getRuntime() - intakeColorSensorDetectionStartTime);
                    }
                } else {
                    intakeColorSensorDetectionStartTime = Double.NaN;
                    hasIntakeSensorDetected = false;
                    telemetry.addData("intakeColorSensor", "Not Detected");
                }

                // Check middle color sensor
                if (((DistanceSensor) middleColorSensor).getDistance(DistanceUnit.CM) <= INTAKE_DISTANCE) {
                    if (Double.isNaN(middleColorSensorDetectionStartTime)) {
                        middleColorSensorDetectionStartTime = getRuntime();
                    }
                    if (getRuntime() - middleColorSensorDetectionStartTime >= DETECTION_HOLD_TIME) {
                        telemetry.addData("middleColorSensor", "Detected");
                        hasMiddleSensorDetected = true;
                    } else {
                        telemetry.addData("middleColorSensor", "Detecting... %.1fs", getRuntime() - middleColorSensorDetectionStartTime);
                    }
                } else {
                    middleColorSensorDetectionStartTime = Double.NaN;
                    hasMiddleSensorDetected = false;
                    telemetry.addData("middleColorSensor", "Not Detected");
                }

                // Check turret color sensor
                if (((DistanceSensor) turretColorSensor).getDistance(DistanceUnit.CM) <= INTAKE_DISTANCE) {
                    if (Double.isNaN(turretColorSensorDetectionStartTime)) {
                        turretColorSensorDetectionStartTime = getRuntime();
                    }
                    if (getRuntime() - turretColorSensorDetectionStartTime >= DETECTION_HOLD_TIME) {
                        telemetry.addData("turretColorSensor", "Detected");
                        hasTurretSensorDetected = true;
                    } else {
                        telemetry.addData("turretColorSensor", "Detecting... %.1fs", getRuntime() - turretColorSensorDetectionStartTime);
                    }
                } else {
                    turretColorSensorDetectionStartTime = Double.NaN;
                    hasTurretSensorDetected = false;
                    telemetry.addData("turretColorSensor", "Not Detected");
                }

                telemetry.addData("All Sensors Detected", hasIntakeSensorDetected && hasMiddleSensorDetected && hasTurretSensorDetected);

                // Only stop intake if all sensors detect for 1+ second AND auto-stop is allowed
                if (intakeMode == IntakeModes.FORWARD && hasIntakeSensorDetected && hasMiddleSensorDetected && hasTurretSensorDetected) {
                    intakeMode = IntakeModes.FULL;
                    setLedGreen();
                    telemetry.addData("AUTO STOP", "All sensors detected!");
                }

                // ODOMETRY RESET BUTTON - Reset to human player starting position
                if (gamepad1.backWasPressed()) {
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
                    } /*else {
                    // Default to (0,0) if alliance unknown
                    resetX = 0;
                    resetY = 0;
                    telemetry.addLine("ODOMETRY RESET: Default Position (0, 0)");
                } */

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
                    telemetry.addLine("ALLIANCE SET TO RED!");
                } else if (gamepad2.x && !isReadingAprilTag) {
                    // ALIGN TO BLUE GOAL
                    autoAlliance = "BLUE";
                    targetGoalTagId = APRILTAG_ID_GOAL_BLUE;
                    telemetry.addLine("ALLIANCE SET TO BLUE!");
                }

                //TURRET STATE CHANGE CONTROLS
                if (gamepad2.left_trigger > 0.1) {
                    turretFSM.setState(TurretFSM.TurretStates.ODOMETRY);
                    shootingPowerMode = ShootingPowerModes.ODOMETRY;
                } else if (gamepad2.right_trigger > 0.1) {
                    turretFSM.setState(TurretFSM.TurretStates.CAMERA);
                    startReadingGoalId();
                }

                // -----------------
                // IMPORTANT: ALWAYS PUT MACRO CONTROLS AFTER MANUAL CONTROLS
                // -----------------

                //CONTROL: START SHOTGUN MACRO USING FSM
                if (gamepad2.dpad_down && gamepad2.right_stick_y < -0.05) {
                    shootArtifactFSM.startShooting(SHOT_GUN_POWER_UP_FAR);
                    shotStartTime = getRuntime();
                    shotStarted = true;
                } else if (gamepad2.dpad_down) {
                    shootArtifactFSM.startShooting(SHOT_GUN_POWER_UP);
                    shotStartTime = getRuntime();
                    shotStarted = true;
                }


            } //manual controls
            else {
                // -----------------
                // MACRO CONTROLS
                // -----------------

                //CONTROL: SHOTGUN MACRO
                if (shotStarted)  {
                    // ONLY UPDATE IF IN MACRO CONTROL MODE
                    shootArtifactFSM.updateShooting();
                    if (shootArtifactFSM.shootingDone() || getRuntime() - shotStartTime >= SHOT_TIMEOUT) {
                        shootArtifactFSM.resetShooting();
                        shotStarted = false;
                    }
                }

                /*
                telemetry.addData("shotStarted", shotStarted);
                telemetry.addData("shootingStage", shootArtifactFSM.getShootingStage());
                telemetry.addData("shootingDone", shootArtifactFSM.shootingDone());

                 */

            } //macro controls

            // Get current robot pose from follower
            robotX = follower.getPose().getX();
            robotY = follower.getPose().getY();
            robotHeadingRadians = follower.getPose().getHeading();

            // TURRET CONTROLS
            if (gamepad2.left_stick_x <= -0.05) {
                turretFSM.rotateLeft();
                turretFSM.setState(TurretFSM.TurretStates.MANUAL);
            } else if (gamepad2.left_stick_x >= 0.05) {
                turretFSM.rotateRight();
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
                if (robotY <= 48) {
                    shotgunPowerLatch = ShotgunPowerLevel.HIGH;
                } else {
                    shotgunPowerLatch = ShotgunPowerLevel.LOW;
                }
            }

            //Latch control
                if (gamepad2.right_stick_y < -.05) {
                    shotgunPowerLatch = ShotgunPowerLevel.HIGH;
                    shootingPowerMode = ShootingPowerModes.MANUAL;
                } else if (gamepad2.right_stick_y > 0.05) {
                    shotgunPowerLatch = ShotgunPowerLevel.LOW;
                    shootingPowerMode = ShootingPowerModes.MANUAL;
                } else if (gamepad2.rightStickButtonWasPressed()) {
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
                    default:
                    case LOW:
                        shotgunFSM.toPowerUp(SHOT_GUN_POWER_UP_RPM);
                        telemetry.addData("Requested ShotGun RPM", SHOT_GUN_POWER_UP_RPM);
                        break;
                }
            telemetry.addData("Actual ShotGun RPM", ejectionMotor.getVelocity() * 60 / TICKS_PER_ROTATION); // convert from ticks per second to RPM
            telemetry.addData("ejectionMotor power", ejectionMotor.getPower());
            telemetry.addData("Actual ShotGun TPS", ejectionMotor.getVelocity()); // convert from ticks per second to RPM

            /*
            telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.04f",
                    pidfOrig.p, pidfOrig.i, pidfOrig.d, pidfOrig.f);
            telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.04f",
                    pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);

             */

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
            //THIS IS FALLBACK FOR ODOMETRY MODE WHEN CAMERA DOES NOT WORK (ai gen untested)
        /*} else {
            // No detection found - check if we should fall back to odometry
            if (turretState == TurretStates.CAMERA &&
                    getRuntime() - lastCameraDetectionTime >= (DarienOpModeFSM.CAMERA_FALLBACK_TIMEOUT_MS / 1000.0)) {
                telemetry.addLine("Goal Detection: Camera timeout - FALLING BACK TO ODOMETRY!");
                turretState = TurretStates.ODOMETRY;
            }
             */

        } // end tagFSM is done
        isCalculatingTurretTargetPosition = false;
    }

} //TeleOpFSM class

