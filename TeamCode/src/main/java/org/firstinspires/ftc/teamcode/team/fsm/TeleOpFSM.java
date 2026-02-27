package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    private enum TurretStates {MANUAL, AUTO}
    private TurretStates turretState = TurretStates.MANUAL;

    private enum IntakeModes {OFF, FORWARD, REVERSE, FULL, SHOOT}
    private IntakeModes intakeMode = IntakeModes.OFF;

    private enum GateStates {CLOSED, OPEN}

    private GateStates gateState = GateStates.CLOSED;

    // AUTOMATIC TURRET CONTROLS BASED ON CAMERA APRILTAG DETECTION
    AprilTagDetection detection;
    double yaw, range; // Stores detection.ftcPose.yaw
    double currentHeadingDeg;
    double relativeHeadingDeg; // Camera-relative bearing to AprilTag (degrees)
    double targetServoPos = Double.NaN; // Convert heading → servo position
    double rawBearingDeg; // Stores detection.ftcPose.bearing;

    // cameraOffsetX < 0 if camera is mounted on the LEFT
   // public static double cameraOffsetX = 0.105; // in centimeter, positive is right, negative is left
    double correctedBearingRad;
    double correctedBearingDeg;
    boolean isCalculatingTurretTargetPosition = false;

    int targetGoalTagId;
    double turretOffset;
    private String autoAlliance = "UNKNOWN";


    //private clamp test
    private static double clampT(double v, double min, double max) {
        if (Double.isNaN(v)) return min;               // defensive: treat NaN as min
        if (min > max) {                               // tolerate inverted bounds
            double t = min; min = max; max = t;
        }
        return Math.max(min, Math.min(max, v));
    }

    @Override
    public void initControls() {
        super.initControls();
        follower = Constants.createFollower(hardwareMap);
        gateServo.setPosition(GATE_CLOSED);
        gateState = GateStates.CLOSED;
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
            turretOffset = TURRET_OFFSET_BLUE;
        } else if ("RED".equals(autoAlliance)) {
            targetGoalTagId = APRILTAG_ID_GOAL_RED;
            turretOffset = TURRET_OFFSET_RED;
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

            if (isReadingAprilTag) {
                updateReadingGoalId();
            }

            // -----------------
            // GAMEPAD1 CONTROLS
            // -----------------

            //RubberBands + topIntake CONTROLS:
            if (gamepad1.y) {
                // Intake on
                intakeMode = IntakeModes.FORWARD;
                gateState = GateStates.CLOSED;
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
                gateState = GateStates.CLOSED;
            } else if (gamepad2.right_bumper) {
                gateState = GateStates.OPEN;
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
                    gateState = GateStates.CLOSED;
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

            // GATE CONTROL
            switch (gateState) {
                case CLOSED:
                    gateServo.setPosition(GATE_CLOSED);
                    break;
                case OPEN:
                default:
                    gateServo.setPosition(GATE_OPEN);
                    break;
            }

            // Add debug telemetry
            telemetry.addData("Gate", gateState);
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
            }


            // -----------------
            // GAMEPAD2 CONTROLS
            // -----------------

            if (!shotStarted) {
                // -----------------
                // MANUAL CONTROLS: only allowed when not running macros
                // -----------------

                //CONTROL: POINT TURRET TO GOAL
                if (gamepad2.b && !isReadingAprilTag) {
                    // ALIGN TO RED GOAL
                    tagFSM.start(getRuntime());
                    isReadingAprilTag = true;
                    targetGoalTagId = APRILTAG_ID_GOAL_RED;
                    telemetry.addLine("ALIGN TURRET TO RED!");
                } else if (gamepad2.x && !isReadingAprilTag) {
                    // ALIGN TO BLUE GOAL
                    tagFSM.start(getRuntime());
                    isReadingAprilTag = true;
                    targetGoalTagId = APRILTAG_ID_GOAL_BLUE;
                    telemetry.addLine("ALIGN TURRET TO BLUE!");
                } else if (isReadingAprilTag) {
                    updateReadingGoalId();
                } // end if (isReadingAprilTag)

                displayTurretTelemetry(yaw, rawBearingDeg, currentHeadingDeg, currentTurretPosition, targetServoPos, range);


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

            //turret rotation
            if (gamepad2.left_stick_x <=-0.05) {    //turn turret clockwise
                //updating the current turret position to be in range of the min and max
                currentTurretPosition = clampT(currentTurretPosition + TURRET_ROTATION_INCREMENT, TURRET_ROTATION_MAX_LEFT, TURRET_ROTATION_MAX_RIGHT);
                //sets turret position
                turretServo.setPosition(currentTurretPosition);
                turretState = TurretStates.MANUAL;
            }
            else if (gamepad2.left_stick_x >= 0.05) {   //turn turret counterclockwise
                //updating the current turret position to be in range of the min and max
                currentTurretPosition = clampT(currentTurretPosition - TURRET_ROTATION_INCREMENT, TURRET_ROTATION_MAX_LEFT, TURRET_ROTATION_MAX_RIGHT);
                //sets turret position
                turretServo.setPosition(currentTurretPosition);
                turretState = TurretStates.MANUAL;
            }
           /* else if (gamepad2.leftStickButtonWasPressed() && !Double.isNaN(targetServoPos)) {
                currentTurretPosition = targetServoPos;
                turretServo.setPosition(targetServoPos);
            }

            */
            /*    telemetry.addData("TurretPos", "%.3f", currentTurretPosition);
                telemetry.addData("Turret Min/Max/Inc", "%.3f / %.3f / %.3f", TURRET_ROTATION_MIN, TURRET_ROTATION_MAX, TURRET_ROTATION_INCREMENT); */

            //CONTROL: EJECTION MOTORS
            //if (!trayFSM.isAutoIntakeRunning()) {
                //Latch control
                if (gamepad2.right_stick_y < -.05) {
                    shotgunPowerLatch = ShotgunPowerLevel.HIGH;
                } else if (gamepad2.right_stick_y > 0.05) {
                    shotgunPowerLatch = ShotgunPowerLevel.LOW;
                } else if (gamepad2.rightStickButtonWasPressed()) {
                    shotgunPowerLatch = ShotgunPowerLevel.OFF;
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

            // Display alliance color from SharedPreferences
            telemetry.addData("Alliance Color from Auto", autoAlliance);
            telemetry.addData("Target AprilTag ID", targetGoalTagId);

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
                turretOffset = TURRET_OFFSET_RED;
            } else if (targetGoalTagId == APRILTAG_ID_GOAL_BLUE) {
                aprilTagDetections.removeIf(tag -> tag.id == 24 || tag.id == 21 || tag.id == 22 || tag.id == 23);
                turretOffset = TURRET_OFFSET_BLUE;
            }
            if (!aprilTagDetections.isEmpty()) {
                telemetry.addLine("Goal Detection: FOUND APRILTAG!");
                // Rotate the turret only if an apriltag is detected and it's the target goal apriltag id
                detection = aprilTagDetections.get(0);
                if (detection.id == targetGoalTagId && detection.ftcPose != null) {
                    telemetry.addLine("Goal Detection: ALIGNING TURRET TO GOAL " + targetGoalTagId);
                    yaw = detection.ftcPose.yaw; // TODO: REMOVE LATER SINCE IT'S ONLY FOR TELEMETRY
                    range = detection.ftcPose.range; // TODO: REMOVE LATER SINCE IT'S ONLY FOR TELEMETRY

                    // Current turret heading (degrees)
                    currentHeadingDeg = turretFSM.getTurretHeading(); // TODO: REMOVE LATER SINCE IT'S ONLY FOR TELEMETRY

                    // Camera-relative bearing to AprilTag (degrees)
                    rawBearingDeg = detection.ftcPose.bearing;

                    if (!isCalculatingTurretTargetPosition) {
                        isCalculatingTurretTargetPosition = true;
                        targetServoPos = TURRET_POSITION_CENTER + turretOffset + RATIO_BETWEEN_TURRET_GEARS * rawBearingDeg / FIVE_ROTATION_SERVO_SPAN_DEG;
                        if (!Double.isNaN(targetServoPos)) {
                            currentTurretPosition = targetServoPos;
                            turretServo.setPosition(targetServoPos);
                        }
                    }
                } else if (detection.ftcPose == null) {
                    telemetry.addLine("Goal Detection: WARNING - Pose estimation failed!");
                } // end detection.id == 20 or 24
            } // end detection is empty
        } // end tagFSM is done
        isCalculatingTurretTargetPosition = false;
    }

    private void displayTurretTelemetry(double yaw, double rawBearingDeg, double currentHeadingDeg, double currentTurretPosition, double targetServoPos, double distanceFromGoal) {
        telemetry.addData("yaw", yaw);
        telemetry.addData("Raw Bearing Deg (alpha)", rawBearingDeg);
        telemetry.addData("currentHeadingDeg (C0)", currentHeadingDeg);
        telemetry.addData("currentTurretPosition", currentTurretPosition);
        telemetry.addData("targetServoPos", targetServoPos);
        telemetry.addData("range", distanceFromGoal);
        // DO NOT ADD telemetry.update() HERE
    }

} //TeleOpFSM class

