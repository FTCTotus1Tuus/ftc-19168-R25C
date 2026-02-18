package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.team.MotorHelper;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

/**
 * Base OpMode for Pedro pathing and state machine logic.
 * Extend this class for autonomous OpModes using Pedro pathing.
 */
@Config
@Configurable
public abstract class DarienOpModeFSM extends LinearOpMode {

    // Pedro pathing/state machine FSMs (declare as needed)
    // public PathFollowerFSM pathFollowerFSM;
    public AprilTagDetectionFSM tagFSM;
    public ShootPatternFSM shootPatternFSM;
    public ShootArtifactFSM shootArtifactFSM;
    public TrayFSM trayFSM;
    public ShootTripleFSM shootTripleFSM;
    public ShotgunFSM shotgunFSM;
    public TurretFSM turretFSM;
    public MotorHelper MotorHelper;

    // AprilTag
    public ArrayList<AprilTagDetection> aprilTagDetections;
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal = null;

    // telemetry
    public TelemetryPacket tp;
    public FtcDashboard dash;

    // HARDWARE DEVICES
    public Servo TrayServo, Elevator, turretServo;
    public CRServo topIntake, rightIntake, leftIntake;
    public DcMotorEx ejectionMotor, rubberBands;
    public DigitalChannel ledRightGreen, ledLeftGreen, ledRightRed, ledLeftRed;

    public NormalizedColorSensor intakeColorSensor;

    // HARDWARE FIXED CONSTANTS
    public static final double encoderResolution = 537.7; //no change unless we change motors
    public static final double wheelDiameter = 3.75; // inches
    public static final double constMult = (wheelDiameter * (Math.PI));
    public static final double inchesToEncoder = encoderResolution / constMult;
    public static final double PI = 3.1416;
    public static final double TICKS_PER_ROTATION = 28*4; // for goBILDA 6000 rpm motor 5203. Each rotation has 28 ticks, and with 4x encoder mode, it's 28*4.
    public static final int FIVE_ROTATION_SERVO_SPAN_DEG = 1800; // Degrees of rotation (5-rotation goBILDA servo)
    public static final int RATIO_BETWEEN_TURRET_GEARS = 6;

    // HARDWARE TUNING CONSTANTS
    public static double TRAY_SERVO_DURATION_ROTATE = 1.5; // seconds
    public static double TRAY_POS_1_INTAKE = 0.279;//275
    public static double TRAY_POS_2_INTAKE = 0.210;//205
    public static double TRAY_POS_3_INTAKE = 0.355;
    public static double TRAY_POS_1_SCORE = 0.172;
    public static double TRAY_POS_2_SCORE = 0.245;
    public static double TRAY_POS_3_SCORE = 0.318;
    public static double TRAY_POS_3_SCORE_GPP = 0.103;
    public static final double ELEVATOR_POS_UP = 0.85;
    public static final double ELEVATOR_POS_DOWN = 0.45;
    public static double SHOT_GUN_POWER_UP = 0.60;
    public static double SHOT_GUN_POWER_UP_FAR = 0.64;//66
    public static double SHOT_GUN_POWER_UP_RPM = 700; // tuned to 6000 rpm motor
    public static double SHOT_GUN_POWER_UP_RPM_AUTO = 650;
    public static double SHOT_GUN_POWER_UP_FAR_RPM_AUTO = 760;// tuned to 6000 rpm motor
    public static double SHOT_GUN_POWER_UP_FAR_RPM_TELEOP = 850; // tuned to 6000 rpm motor
    public static double SHOT_GUN_POWER_DOWN = 0.2; // tuned to 6000 rpm motor

    // For FTC AprilTag detection with a Logitech C910 webcam,
    // 1.0 to 1.5 seconds is typically sufficient and more reasonable than 3 seconds.
    // Recommended timeout values:
    //   1.0 second (1000ms): Good for typical autonomous scenarios where the camera has a clear view of tags
    //   1.5 seconds (1500ms): More conservative, allows for slight delays in camera initialization or processing
    //   0.5 seconds (500ms): Can work if tags are large, close, and well-lit, but may be too aggressive
    // Factors to consider:
    //   Camera exposure: The C910 may need 1-2 frames to adjust exposure in varying light conditions
    //   Processing time: AprilTag detection typically runs at 10-30 FPS, so 1 second gives 10-30 detection attempts
    //   Distance/size: Tags further away or smaller may need slightly more time
    //   Lighting: Poor lighting may require longer timeout
    public static double TIMEOUT_APRILTAG_DETECTION = 0.75; // seconds

    public static double INTAKE_RUBBER_BANDS_POWER = 1;
    public static double OUTPUT_RUBBER_BANDS_POWER = 0.3;
    public static double INTAKE_INTAKE_ROLLER_POWER = 1;
    public static double OUTPUT_INTAKE_ROLLER_POWER = 0.2;
    public static double TURRET_ROTATION_INCREMENT = 0.002;
    public static double TURRET_ROTATION_MAX_LEFT = 0.63;
    public static double TURRET_ROTATION_MAX_RIGHT = 0.35;
    public static double TURRET_POSITION_CENTER = 0.5;

    // PIDF Constants for DcMotorEx.setVelocityPIDFCoefficients()
    public static double EJECTION_P=15;
    public static double EJECTION_I=3;
    public static double EJECTION_D=0;
    public static double EJECTION_F=12.5;

    // PID Constants for custom MotorHelper PID functions
    public static double SHOT_GUN_PGAIN = 0.015;
    public static double SHOT_GUN_PGAIN2 = 0.015;
    public static double SHOT_GUN_IGAIN = 0.0002;
    public static double SHOT_GUN_PDUTY_MIN = -0.5;
    public static double SHOT_GUN_PDUTY_MAX = 1;
    public static double SHOT_GUN_IDUTY_MIN = 0;
    public static double SHOT_GUN_IDUTY_MAX = 1;
    public static double SHOT_GUN_POWER_MIN = 0;
    public static double SHOT_GUN_POWER_MAX = 1;
    public static double SHOT_GUN_GAIN = 1;
    public static double SHOT_GUN_MIN_RPM = 0;
    public static double SHOT_GUN_MAX_RPM = 1000;

    public final int APRILTAG_ID_GOAL_BLUE = 20;
    public final int APRILTAG_ID_GOAL_RED = 24;
    public static double TURRET_OFFSET_RED = 0.005;
    public static double TURRET_OFFSET_BLUE = 0.01;

    // CAMERA EXPOSURE/GAIN SETTINGS FOR APRILTAG DETECTION
    // Low exposure (5-6ms) with high gain reduces motion blur and improves far-distance detection
    // Tune these values using the TuneAprilTagExposure OpMode while viewing camera stream
    public static int APRILTAG_EXPOSURE_MS = 6;  // Milliseconds (lower = less blur, start with 5-6)
    public static int APRILTAG_GAIN = 255;       // 0-255 (higher = brighter in low light, start at max)

    // DYNAMIC VARIABLES
    public double currentTrayPosition;
    public double currentTurretPosition;

    public int targetGoalId = 0;
    public enum ShotgunPowerLevel {
        OFF,
        LOW,
        HIGH
    }

    // Abstract method for child classes to implement
    @Override
    public abstract void runOpMode() throws InterruptedException;

    public ServoIncrementalFSM intakeServoFSM;
    //public ServoIncrementalFSM trayServoFSM;

    public void initControls() {

        //TELEMETRY
        // TODO: Put a flag to turn on/off ftc dashboard. We don't want that to run during matches.
        tp = new TelemetryPacket();
        dash = FtcDashboard.getInstance();

        // INITIALIZE SERVOS
        TrayServo = hardwareMap.get(Servo.class, "Tray");
        Elevator = hardwareMap.get(Servo.class, "Elevator");
        topIntake = hardwareMap.get(CRServo.class, "topIntake");
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");
        leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
        turretServo.setPosition(TURRET_POSITION_CENTER); // set to center position
        currentTurretPosition = TURRET_POSITION_CENTER;
        //turretServo.scaleRange(TURRET_ROTATION_MAX_RIGHT, TURRET_ROTATION_MAX_LEFT); // limit the servo range to the turret rotation range

        // INITIALIZE SENSORS
        intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeColorSensor");

        // INITIALIZE MOTORS
        rubberBands = hardwareMap.get(DcMotorEx.class, "rubberBands");
        rubberBands.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rubberBands.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        ejectionMotor = hardwareMap.get(DcMotorEx.class, "ejectionMotor");
        ejectionMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        ejectionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //ejectionMotor.setVelocityPIDFCoefficients(EJECTION_P, EJECTION_I, EJECTION_D, EJECTION_F);

        //INITIALIZE LED
        ledRightGreen = hardwareMap.get(DigitalChannel.class, "LEDRight1");
        ledLeftGreen = hardwareMap.get(DigitalChannel.class, "LEDLeft1");
        ledRightRed = hardwareMap.get(DigitalChannel.class, "LEDRight2");
        ledLeftRed = hardwareMap.get(DigitalChannel.class, "LEDLeft2");
        setLedRed();


        initAprilTag();

        MotorHelper = new MotorHelper(telemetry, TICKS_PER_ROTATION);

        // INSTANTIATE THE STATE MACHINES
        tagFSM = new AprilTagDetectionFSM(aprilTag, TIMEOUT_APRILTAG_DETECTION);
        shootArtifactFSM = new ShootArtifactFSM(this);
        shootPatternFSM = new ShootPatternFSM(this);
        trayFSM = new TrayFSM(this, TrayServo, rubberBands, topIntake, rightIntake, leftIntake, intakeColorSensor, telemetry, ledRightGreen, ledLeftGreen, ledRightRed, ledLeftRed);
        shootTripleFSM = new ShootTripleFSM(this);
        shotgunFSM = new ShotgunFSM(SHOT_GUN_POWER_UP, SHOT_GUN_POWER_UP_FAR, ejectionMotor, this, MotorHelper);
        turretFSM = new TurretFSM(this);

        //trayServoFSM = new ServoIncrementalFSM(TrayServo);
        //currentTrayPosition = TRAY_POS_1_SCORE; // set a default tray position


        telemetry.addLine("FTC 19168 Robot Initialization Done!");
        telemetry.update();
    }

    /**
     * Set the tray position and update the currentTrayPosition variable.
     *
     * @param position The desired position for the tray servo.
     * @param duration The duration over which to move the tray servo to the desired position.
     */
    public void setTrayPosition(double position, double duration) {
        TrayServo.setPosition(position);
        //servoIncremental(TrayServo, position, currentTrayPosition, duration, 4);
        currentTrayPosition = position;
        /*
        if (!trayServoFSM.isRunning()) {
            targetTrayPosition = position;
            trayServoFSM.start(position, currentTrayPosition, duration, getRuntime());
        }
         */
    }

    /**
     * Set the tray position with a default duration in seconds.
     *
     * @param position The desired position for the tray servo.
     */
    public void setTrayPosition(double position) {
        setTrayPosition(position, TRAY_SERVO_DURATION_ROTATE);
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

        while (currentTime - startTime < endDuration) {
            if (endPos > startPos) {
                // rotate tray clockwise
                currentPos = ((endPos - startPos) / (endDuration - (currentTime - startTime))) * (currentTime - startTime) + startPos;
            } else {
                // rotate tray counterclockwise
                currentPos = ((startPos - endPos) / (endDuration - (currentTime - startTime))) * (currentTime - startTime) + endPos;
            }
            servo.setPosition(currentPos / divisor);

            //telemetry.addData("currentPos:", currentPos);
            //telemetry.addData("currentTime:", currentTime);
            //telemetry.update();
            //tp.put("currentServo", currentPos);
            //tp.put("currentTime", currentTime);

            //dash.sendTelemetryPacket(tp);

            if (currentPos >= endPos) {
                return;
            }
            currentTime = getRuntime();
        }
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

        // Set manual exposure and gain to reduce motion blur and improve detection at distance
        // This is especially important for detecting AprilTags from far positions
        //setManualExposure(APRILTAG_EXPOSURE_MS, APRILTAG_GAIN);
    }

    /**
     * Manually set the camera gain and exposure for AprilTag detection.
     * Low exposure (5-6ms) with high gain reduces motion blur.
     * Can only be called AFTER calling initAprilTag().
     *
     * @param exposureMS Camera exposure time in milliseconds
     * @param gain       Camera gain value (typically 0-255)
     * @return true if controls are set successfully
     */
    private boolean setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting for stream...");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping
        if (!isStopRequested()) {
            try {
                // Set exposure - must be in Manual Mode for these values to take effect
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
                sleep(20);

                // Set Gain
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(gain);
                sleep(20);

                telemetry.addData("Camera Exposure", exposureMS + "ms");
                telemetry.addData("Camera Gain", gain);
                telemetry.update();
                return true;
            } catch (Exception e) {
                telemetry.addData("Camera Control Error", e.getMessage());
                telemetry.update();
                return false;
            }
        } else {
            return false;
        }
    }

    /**
     * Read this camera's minimum and maximum Exposure and Gain settings.
     * Useful for tuning and debugging.
     * Can only be called AFTER calling initAprilTag().
     *
     * @return int array: [minExposure, maxExposure, minGain, maxGain]
     */
    public int[] getCameraSettings() {
        // Ensure Vision Portal has been setup
        if (visionPortal == null) {
            return new int[]{0, 0, 0, 0};
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting for stream...");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
        }

        // Get camera control values unless we are stopping
        if (!isStopRequested()) {
            try {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                int minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
                int maxExposure = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                int minGain = gainControl.getMinGain();
                int maxGain = gainControl.getMaxGain();

                return new int[]{minExposure, maxExposure, minGain, maxGain};
            } catch (Exception e) {
                telemetry.addData("Camera Settings Error", e.getMessage());
                telemetry.update();
                return new int[]{0, 0, 0, 0};
            }
        }
        return new int[]{0, 0, 0, 0};
    }

    /**
     * Adjust motor power based on current battery voltage to maintain consistent performance.
     *
     * @param power The desired motor power (range -1.0 to 1.0).
     * @return The adjusted motor power.
     */
    public double getVoltageAdjustedMotorPower(double power) {
        double nominalVoltage = 13.0; // Typical full battery voltage for FTC
        double currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        double scale = nominalVoltage / currentVoltage;
        return power * scale;
    }

    public double getTicksPerSecond(double requestedRPM) {
        try {
            double targetRPM = requestedRPM;
            double ticksPerSecond = (targetRPM / 60.0) * TICKS_PER_ROTATION;
            return ticksPerSecond;
        }
        catch (Exception e) {
            // telemetry.addData("Ticks/Sec Adjustment Error", e.getMessage());
            return requestedRPM; // if error, return requested power unmodified
        }

    }

    public double getRpmFromTicksPerSecond(double ticksPerSecond) {
        try {
            double rpm = (ticksPerSecond * 60.0) / TICKS_PER_ROTATION;
            return rpm;
        } catch (Exception e) {
            // telemetry.addData("RPM from Ticks/Sec Error", e.getMessage());
            return ticksPerSecond; // if error, return requested power unmodified
        }

    }

    public void setLedRed() {
        ledLeftGreen.setMode(DigitalChannel.Mode.OUTPUT);
        ledLeftGreen.setState(false); // LOW turns LED on (typically)
        ledLeftRed.setMode(DigitalChannel.Mode.OUTPUT);
        ledLeftRed.setState(true);
        ledRightGreen.setMode(DigitalChannel.Mode.OUTPUT);
        ledRightGreen.setState(false);
        ledRightRed.setMode(DigitalChannel.Mode.OUTPUT);
        ledRightRed.setState(true);
    }

    public void setLedGreen() {
        ledLeftGreen.setMode(DigitalChannel.Mode.OUTPUT);
        ledLeftGreen.setState(true); // HIGH turns LED off (typically)
        ledLeftRed.setMode(DigitalChannel.Mode.OUTPUT);
        ledLeftRed.setState(false);
        ledRightGreen.setMode(DigitalChannel.Mode.OUTPUT);
        ledRightGreen.setState(true);
        ledRightRed.setMode(DigitalChannel.Mode.OUTPUT);
        ledRightRed.setState(false);
    }

    public void setLedAmber() {
        ledLeftGreen.setMode(DigitalChannel.Mode.OUTPUT);
        ledLeftGreen.setState(false);
        ledLeftRed.setMode(DigitalChannel.Mode.OUTPUT);
        ledLeftRed.setState(false);
        ledRightGreen.setMode(DigitalChannel.Mode.OUTPUT);
        ledRightGreen.setState(false);
        ledRightRed.setMode(DigitalChannel.Mode.OUTPUT);
        ledRightRed.setState(false);
    }

    public void setLedOff() {
        ledLeftGreen.setMode(DigitalChannel.Mode.OUTPUT);
        ledLeftGreen.setState(true); // HIGH turns LED off (typically)
        ledLeftRed.setMode(DigitalChannel.Mode.OUTPUT);
        ledLeftRed.setState(true);
        ledRightGreen.setMode(DigitalChannel.Mode.OUTPUT);
        ledRightGreen.setState(true);
        ledRightRed.setMode(DigitalChannel.Mode.OUTPUT);
        ledRightRed.setState(true);
    }

    //Red when start
    //amber when intaking
    //green when done intaking
    //amber when shooting
    //red when done shooting

    /** Clamp a value between a minimum and maximum.
     *
     * @param val The value to clamp.
     * @param min The minimum value.
     * @param max The maximum value.
     * @return if val<min = min,if min<val<max = val, if val>max = max
     */
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    //goal: automate intake as much as possible, save tray positions, checking if there are balls in either of 3 slots, not just yes not but color aswell, automate intake, sensor checks if ball is in, then rotate, intake etc...
   /* public void intakeColorSensorTelemetry() {
        if (intakeColorSensor == null) {
            telemetry.addLine("Intake Color Sensor: not configured");
            return;
        }

        // Read once and reuse to keep values consistent
        com.qualcomm.robotcore.hardware.NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();

        // Convert normalized 0.0-1.0 channels to 0-255 ints (clamped and rounded)
        int r255 = (int) (clamp(colors.red, 0.0, 1.0) * 255.0 + 0.5);
        int g255 = (int) (clamp(colors.green, 0.0, 1.0) * 255.0 + 0.5);
        int b255 = (int) (clamp(colors.blue, 0.0, 1.0) * 255.0 + 0.5);
        int a255 = (int) (clamp(colors.alpha, 0.0, 1.0) * 255.0 + 0.5);

        // Compute HSV from 0-255 RGB
        float[] hsvValues = new float[3];
        android.graphics.Color.RGBToHSV(r255, g255, b255, hsvValues);

        // Telemetry: normalized and 0-255 values plus hue
        telemetry.addData("Intake Color - Alpha (0-1)", colors.alpha);
        telemetry.addData("Intake Color - Alpha (0-255)", a255);
        telemetry.addData("Intake Color - Red (0-1)  ", colors.red);
        telemetry.addData("Intake Color - Red (0-255)", r255);
        telemetry.addData("Intake Color - Green (0-1)", colors.green);
        telemetry.addData("Intake Color - Green (0-255)", g255);
        telemetry.addData("Intake Color - Blue (0-1) ", colors.blue);
        telemetry.addData("Intake Color - Blue (0-255) ", b255);
        telemetry.addData("Intake Color - Hue (deg)   ", hsvValues[0]);
    }*/
}
