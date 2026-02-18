// To run on the robot:
// 1. Make Module   (Top ribbon)
// 2. Run           (Top ribbon)

package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Config
public class DarienOpMode extends LinearOpMode {

    // AprilTag
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal = null;        // Used to manage the video source.
    // telemetry
    public TelemetryPacket tp;
    public FtcDashboard dash;
    // HARDWARE COMPONENTS
    public DcMotor omniMotor0; // left front
    public DcMotor omniMotor1; // right front
    public DcMotor omniMotor2; // left rear
    public DcMotor omniMotor3; // right rear
   // public DcMotor slideMotor1;
   // public DcMotor tiltMotor;
   // public Servo claw;
    public Servo TrayServo;
    public Servo Elevator;
    public Servo Feeder;
    public Servo IntakeServo;
    public DcMotor ejectionMotorRight;
    public DcMotor ejectionMotorLeft;
    public NormalizedColorSensor intakeColorSensor;
    public CRServo rubberBands;

    //    public IMU imu;
   // public GoBildaPinpointDriver odo;

    // HARDWARE FIXED CONSTANTS
    public static double encoderResolution = 537.7; //no change unless we change motors
    public double wheelDiameter = 3.75; // inches
    public double constMult = (wheelDiameter * (Math.PI));
    public static double rotationTolerance = 2; //in degrees
    public double inchesToEncoder = encoderResolution / constMult;
    public static double PI = 3.1416;


    // HARDWARE TUNING CONSTANTS
    public int encoderPos0, encoderPos1, encoderPos2, encoderPos3;
    public int encoderPos;
    public double regularDivBy = 2;
    public double turboDivBy = 1;
    public boolean turboBoost = false;

    public static double INTAKE_SERVO_POS_UP = 0.75;
    public static double INTAKE_SERVO_POS_DOWN = 0.21;
    public static double INTAKE_SERVO_DURATION_RAISE = 1; // seconds
    public static double TRAY_POS_1_INTAKE = 0.275;
    public static double TRAY_POS_2_INTAKE = 0.205;
    public static double TRAY_POS_3_INTAKE = 0.350;
    public static double TRAY_POS_1_SCORE = 0.385;
    public static double TRAY_POS_2_SCORE = 0.310;
    public static double TRAY_POS_3_SCORE = 0.240;
    double IntakeServoPosition = 0;
    double startTimeIntakeServo = 0;
    boolean isIntakeServoMoving = false;
    public double currentTrayPosition;
    public static double INTAKE_DISTANCE = 5;//in CM
    public static double INTAKE_TIME = 1;
    public static double ELEVATOR_POS_UP = 0.83;
    public static double ELEVATOR_POS_DOWN = 0.45;
    public static double FEEDER_POS_UP = .9;
    public static double FEEDER_POS_DOWN = .45;
    public static double SHOT_GUN_POWER_UP = 0.32; // tuned to 6000 rpm motor
    public static double SHOT_GUN_POWER_UP_FAR = 0.39; // tuned to 6000 rpm motor
    public static double SHOT_GUN_POWER_DOWN = 0.2; // tuned to 6000 rpm motor

    public double TIMEOUT_APRILTAG_DETECTION = 3; // seconds
    public static double INTAKE_RUBBER_BANDS_POWER = -0.7;
    public static double OUTPUT_RUBBER_BANDS_POWER = 0.2;

    double startTimeIntakeColorSensor;
    public boolean intakeLifted = false;
    public double intakeLiftStartTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
    }

    public void initControls() {

        // INITIALIZE SENSORS

        // Initialize 2 Deadwheel odometry
       // configure2DeadWheel();

        //TELEMETRY
        // TODO: Put a flag to turn on/off ftc dashboard. We don't want that to run during matches.
        tp = new TelemetryPacket();
        dash = FtcDashboard.getInstance();

        // INITIALIZE SERVOS
        //claw = hardwareMap.get(Servo.class, "claw");
        TrayServo = hardwareMap.get(Servo.class, "Tray");
        Elevator = hardwareMap.get(Servo.class, "Elevator");
        IntakeServo = hardwareMap.get(Servo.class, "intakeServo");
        Feeder = hardwareMap.get(Servo.class, "Feeder");
        rubberBands = hardwareMap.get(CRServo.class, "rubberBands");
        // INITIALIZE SENSORS
        intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeColorSensor");
        // INITIALIZE MOTORS
        omniMotor0 = initializeMotor("omniMotor0");
        omniMotor1 = initializeMotor("omniMotor1");
        omniMotor2 = initializeMotor("omniMotor2");
        omniMotor3 = initializeMotor("omniMotor3");
        ejectionMotorRight = initializeMotor("ejectionMotorRight");
        ejectionMotorLeft = initializeMotor("ejectionMotorLeft");

        omniMotor0.setDirection(DcMotor.Direction.FORWARD);
        omniMotor1.setDirection(DcMotor.Direction.REVERSE);
        omniMotor2.setDirection(DcMotor.Direction.REVERSE);
        omniMotor3.setDirection(DcMotor.Direction.FORWARD);

        initAprilTag();

        startTimeIntakeColorSensor = getRuntime();
        telemetry.addLine("FTC 19168 Robot Initialization Done!");
        telemetry.update();
    }


    public double getVoltage() {
        return (hardwareMap.voltageSensor.iterator().next().getVoltage());
    }
    public void shotGun(double power) {
        ejectionMotorLeft.setPower(power);
        ejectionMotorRight.setPower(-power);
    }
    public void shotGunStop() {
        ejectionMotorLeft.setPower(0);
        ejectionMotorRight.setPower(0);
        /*double startTime = getRuntime();
        double currentTime = startTime;
        while (currentTime - startTime < seconds) {
            leftMotor.setPower(power);
            rightMotor.setPower(-power);
            currentTime = getRuntime();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);

         */
    }

    public void shootArtifact() {
     Elevator.setPosition(ELEVATOR_POS_UP);
     //shotGun(SHOT_GUN_POWER_DOWN);
     //start spinning down
     sleep(100);
     shotGun(SHOT_GUN_POWER_UP);
     sleep(600);
     //start spinning up
     Feeder.setPosition(FEEDER_POS_UP);
     //move feeder up while spinner is still spinning
     sleep(500);
     shotGunStop();
     //stop spinning
     Feeder.setPosition(FEEDER_POS_DOWN);
     Elevator.setPosition(ELEVATOR_POS_DOWN);
    }

    public void print(String Name, Object message) {
        //saves a line for quick debug messages
        telemetry.addData(Name, message);
        //telemetry.update();
    }

    public double relativePower(double intended_power) {
        //makes sure the power going to the motors is constant over battery life
        return (13 * intended_power) / getVoltage();
    }

    public DcMotor initializeMotor(String name) {
         /*This is just a handy dandy function which saves a few lines and looks cool,
         it initializes the motor and it also initializers the motor power logs for this motor*/
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    public void servoIncremental(Servo servo, double endPos, double startPos, double endDuration, double divisor) {
        //calculate how many increments it will take to reach to position in the target time
        double currentPos;
        double startTime = getRuntime();
        double currentTime = startTime;
        //double Last_Time = currentTime;
        double ActualPos = 0;
        while (currentTime - startTime < endDuration) {
            if(endPos > startPos){
                // rotate tray clockwise
                currentPos = ((endPos - startPos) / (endDuration - (currentTime - startTime))) * (currentTime - startTime) + startPos;
            } else {
                // rotate tray counterclockwise
                currentPos = ((startPos - endPos) / (endDuration - (currentTime - startTime))) * (currentTime - startTime) + endPos;
            }
            servo.setPosition(currentPos/divisor);
            /*
            if (currentTime - Last_Time >= 0.240 ){
                servo.setPosition(currentPos);
                ActualPos = currentPos;
                Last_Time = currentTime;
            }

             */
            telemetry.addData("currentPos:", currentPos);
            telemetry.addData("currentTime:", currentTime);
            telemetry.update();
            tp.put("currentServo",currentPos);
            tp.put("currentTime",currentTime);
            //tp.put("lastTime",Last_Time);
            tp.put("ActPos",ActualPos);

            dash.sendTelemetryPacket(tp);

            if (currentPos >= endPos) {
                //sleep(1000);
                return;
            }
            currentTime = getRuntime();

        }
        //sleep(3000);
    }

    /**
     * Set the tray position and update the currentTrayPosition variable.
     *
     * @param position The desired position for the tray servo.
     * @param duration The duration over which to move the tray servo to the desired position.
     */
    public void setTrayPosition(double position) {
        TrayServo.setPosition(position);
        currentTrayPosition = position;
    }


    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

    }

    protected void telemetryAprilTag() {

        ArrayList<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
    }

    /**
     * Detect apriltag motif
     */
    public ArrayList<AprilTagDetection> readAprilTagSequence() {
        ArrayList<AprilTagDetection> currentDetections = null;
        double startTime = getRuntime();
        do {
            currentDetections = aprilTag.getDetections();
            telemetry.addLine("detecting apriltags...");
            telemetry.update();
        }
        while (currentDetections.isEmpty() || getRuntime() - startTime < TIMEOUT_APRILTAG_DETECTION);
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        telemetry.update();

        return currentDetections;
    }

    /**
     * Shoot the correct pattern by using the readApriltagSequence function
     */
    public void shootApriltagSequence(ArrayList<AprilTagDetection> currentDetections) {
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                switch (detection.id) {
                    case 21:
                        telemetry.addData("Motif", "GPP");
                        // telemetryAprilTag(); //TODO: use this telemetry rather than the line above.
                        // shoot green
                        servoIncremental(TrayServo, TRAY_POS_2_SCORE, currentTrayPosition, 1, 4);
                        currentTrayPosition = TRAY_POS_2_SCORE;
                        sleep(1000);
                        shootArtifact();
                        sleep(500);
                        // shoot purple
                        servoIncremental(TrayServo, TRAY_POS_3_SCORE, currentTrayPosition, 1, 4);
                        currentTrayPosition = TRAY_POS_3_SCORE;
                        sleep(1000);
                        shootArtifact();
                        sleep(500);
                        // shoot purple
                        servoIncremental(TrayServo, TRAY_POS_1_SCORE, currentTrayPosition, 1, 4);
                        currentTrayPosition = TRAY_POS_1_SCORE;
                        sleep(1000);
                        shootArtifact();
                        sleep(500);
                        break;
                    case 22:
                        telemetry.addData("Motif", "PGP");
                        //shoot purple
                        servoIncremental(TrayServo, TRAY_POS_1_SCORE, currentTrayPosition, 1, 4);
                        currentTrayPosition = TRAY_POS_1_SCORE;
                        sleep(1000);
                        shootArtifact();
                        sleep(500);
                        //shoot green
                        servoIncremental(TrayServo, TRAY_POS_2_SCORE, currentTrayPosition, 1, 4);
                        currentTrayPosition = TRAY_POS_2_SCORE;
                        sleep(1000);
                        shootArtifact();
                        sleep(500);
                        //shoot purple
                        servoIncremental(TrayServo, TRAY_POS_3_SCORE, currentTrayPosition, 1, 4);
                        currentTrayPosition = TRAY_POS_3_SCORE;
                        sleep(1000);
                        shootArtifact();
                        sleep(500);
                        break;
                    case 23:
                        telemetry.addData("Motif", "PPG");
                        //shoot purple
                        servoIncremental(TrayServo, TRAY_POS_1_SCORE, currentTrayPosition, 1, 4);
                        currentTrayPosition = TRAY_POS_1_SCORE;
                        sleep(1000);
                        shootArtifact();
                        sleep(500);
                        //shoot purple
                        servoIncremental(TrayServo, TRAY_POS_3_SCORE, currentTrayPosition, 1, 4);
                        currentTrayPosition = TRAY_POS_3_SCORE;
                        sleep(1000);
                        shootArtifact();
                        sleep(500);
                        //shoot green
                        servoIncremental(TrayServo, TRAY_POS_2_SCORE, currentTrayPosition, 1, 4);
                        currentTrayPosition = TRAY_POS_2_SCORE;
                        sleep(1000);
                        shootArtifact();
                        sleep(500);
                        break;
                }

                //return;
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
    }


    // CONTROL: INTAKE
    public void automaticIntake() {
        double startTimeColor = getRuntime();
        /*
        telemetry.addData("Gain", gain);
        telemetry.addLine()
            .addData("Red", "%.3f", colors.red)
            .addData("Green", "%.3f", colors.green)
            .addData("Blue", "%.3f", colors.blue);

     */
        if (intakeColorSensor instanceof DistanceSensor) {
            //telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) intakeColorSensor).getDistance(DistanceUnit.CM));
            if (((DistanceSensor)intakeColorSensor).getDistance(DistanceUnit.CM) <= INTAKE_DISTANCE && (getRuntime()-startTimeColor) >= 1){
                startTimeColor = getRuntime();
                servoIncremental(IntakeServo, INTAKE_SERVO_POS_UP, INTAKE_SERVO_POS_DOWN, 1,1);
            }
        }
        //telemetry.update();
    }

    public double getHypotenuse(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public double getHypotenuse(double x, double y, double z) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public double getVoltageAdjustedMotorPower(double power) {
        double nominalVoltage = 13.0; // Typical full battery voltage for FTC
        double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double scale = nominalVoltage / currentVoltage;
        return power * scale;
    }

}
