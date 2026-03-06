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

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.team.GoBildaPinpointDriver;
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
    public ShotgunFSM shotgunFSM;
    public TurretFSM turretFSM;
    public MotorHelper MotorHelper;
    public GateFSM gateFSM;
    public IntakeFSM intakeFSM;
    public ShootingFSM shootingFSM;

    // AprilTag
    public ArrayList<AprilTagDetection> aprilTagDetections;
    public AprilTagProcessor aprilTag;
    public VisionPortal visionPortal = null;

    // telemetry
    public TelemetryPacket tp;
    public FtcDashboard dash;

    // HARDWARE DEVICES
    public Follower follower;                       // Pedro Pathing follower — created during initControls()
    public DcMotorEx ejectionMotor;

    // HARDWARE FIXED CONSTANTS
    public static final double encoderResolution = 537.7; //no change unless we change motors
    public static final double wheelDiameter = 3.75; // inches
    public static final double constMult = (wheelDiameter * (Math.PI));
    public static final double inchesToEncoder = encoderResolution / constMult;
    public static final double PI = 3.1416;
    public static final double TICKS_PER_ROTATION = 28*4; // for goBILDA 6000 rpm motor 5203. Each rotation has 28 ticks, and with 4x encoder mode, it's 28*4.
    //public static double TICKS_PER_ROTATION = 103.8 * 4; // for goBILDA 1620 rpm motor 5202. Each rotation has 103.8 PPR at the Output Shaft, and with quadrature (4x) encoder mode, it's 103.8 * 4 = 415.2
    public static double ROBOT_CENTER_OFFSET_X = 8.5;
    public static double ROBOT_CENTER_OFFSET_Y = 8.25;

    // HARDWARE TUNING CONSTANTS
    public static double SHOT_GUN_POWER_UP = 0.60;
    public static double SHOT_GUN_POWER_UP_FAR = 0.64;//66
    public static double SHOT_GUN_POWER_UP_RPM = 1100; // tuned to 6000 rpm motor mounted vertically with small bevel gears
    public static double SHOT_GUN_POWER_UP_RPM_AUTO = 1075;
    public static double SHOT_GUN_POWER_UP_FAR_RPM_AUTO = 1350;// tuned to 6000 rpm motor mounted vertically with small bevel gears
    public static double SHOT_GUN_POWER_UP_FAR_RPM_TELEOP = 1350; // tuned to 6000 rpm motor mounted vertically with small bevel gears
    public static double SHOT_GUN_POWER_DOWN = 0.2; // tuned to 6000 rpm motor

    // SHOOTING POWER ODOMETRY TUNING
    public static double SHOOTING_POWER_ODOMETRY_Y_THRESHOLD = 48.0; // Y threshold for automatic FAR vs CLOSE power selection

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

    // PID Constants for custom MotorHelper PID functions
    public static double SHOT_GUN_PGAIN = 0.002;
    public static double SHOT_GUN_PGAIN2 = 0.0005;
    public static double SHOT_GUN_IGAIN = 0.00003;
    public static double SHOT_GUN_PDUTY_MIN = -0.5; // SHOT_GUN_PDUTY_MIN = -0.5 may cause the PID to brake the motor if it overshoots — consider setting it to 0.0 for a flywheel since you never want reverse braking.
    public static double SHOT_GUN_PDUTY_MAX = 1;
    public static double SHOT_GUN_IDUTY_MIN = 0;
    public static double SHOT_GUN_IDUTY_MAX = 1;
    public static double SHOT_GUN_POWER_MIN = 0;
    public static double SHOT_GUN_POWER_MAX = 1;
    public static double SHOT_GUN_GAIN = 1;
    public static double SHOT_GUN_MIN_RPM = 0;
    public static double SHOT_GUN_MAX_RPM = 3000;

    public final int APRILTAG_ID_GOAL_BLUE = 20;
    public final int APRILTAG_ID_GOAL_RED = 24;

    // CAMERA EXPOSURE/GAIN SETTINGS FOR APRILTAG DETECTION
    // Low exposure (5-6ms) with high gain reduces motion blur and improves far-distance detection
    // Tune these values using the TuneAprilTagExposure OpMode while viewing camera stream
    public static int APRILTAG_EXPOSURE_MS = 6;  // Milliseconds (lower = less blur, start with 5-6)
    public static int APRILTAG_GAIN = 255;       // 0-255 (higher = brighter in low light, start at max)

    // FIELD GOAL POSITION CONSTANTS (in inches, Pedro Pathing coordinate system)
    // (0,0) = left audience side (red loading zone), (72,72) = field center, (144,144) = red goal
    public static final double GOAL_RED_X = 144;
    public static final double GOAL_RED_Y = 144;
    public static final double GOAL_BLUE_X = 0;
    public static final double GOAL_BLUE_Y = 144;

    // HUMAN PLAYER POSITION CONSTANTS (in inches, Pedro Pathing coordinate system)
    public static final double HUMAN_PLAYER_RED_X = 0;
    public static final double HUMAN_PLAYER_RED_Y = 0;
    public static final double HUMAN_PLAYER_BLUE_X = 144;
    public static final double HUMAN_PLAYER_BLUE_Y = 0;

    // ODOMETRY AIMING TUNING
    public static double CAMERA_FALLBACK_TIMEOUT_MS = 500; // Auto-switch to odometry after this timeout

    public int targetGoalId = 0;

    public enum ShotgunPowerLevel {
        OFF,
        LOW,
        HIGH
    }

    public enum ShootingPowerModes {
        MANUAL,    // Driver manually selects power level
        ODOMETRY   // Automatic power selection based on robot Y position
    }

    protected ShootingPowerModes shootingPowerMode = ShootingPowerModes.MANUAL;

    // Abstract method for child classes to implement
    @Override
    public abstract void runOpMode() throws InterruptedException;

    public void initControls() {

        //TELEMETRY
        // TODO: Put a flag to turn on/off ftc dashboard. We don't want that to run during matches.
        tp = new TelemetryPacket();
        dash = FtcDashboard.getInstance();

        // INITIALIZE MOTORS
        ejectionMotor = hardwareMap.get(DcMotorEx.class, "ejectionMotor");
        ejectionMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        ejectionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ejectionMotor.setDirection(DcMotorEx.Direction.REVERSE); // Reverse because it is geared

        initAprilTag();

        MotorHelper = new MotorHelper(telemetry, TICKS_PER_ROTATION);

        // Set default shooting power mode: ODOMETRY for Autos, MANUAL for TeleOp
        shootingPowerMode = isAutonomousMode() ? ShootingPowerModes.ODOMETRY : ShootingPowerModes.MANUAL;

        // INSTANTIATE THE STATE MACHINES
        tagFSM = new AprilTagDetectionFSM(aprilTag, TIMEOUT_APRILTAG_DETECTION);
        shootArtifactFSM = new ShootArtifactFSM(this);
        shootPatternFSM = new ShootPatternFSM(this);
        shotgunFSM = new ShotgunFSM(SHOT_GUN_POWER_UP, SHOT_GUN_POWER_UP_FAR, ejectionMotor, this, MotorHelper);
        turretFSM = new TurretFSM(this.hardwareMap);
        turretFSM.init();
        gateFSM = new GateFSM(this.hardwareMap);
        gateFSM.init();
        intakeFSM = new IntakeFSM(this.hardwareMap, this.gateFSM);
        intakeFSM.init();
        shootingFSM = new ShootingFSM(this.gateFSM, this.shotgunFSM, this.intakeFSM, this);

        // Create Pedro Pathing follower — available to all subclasses (Teleop + Autos)
        follower = Constants.createFollower(hardwareMap);

        telemetry.addLine("FTC 19168 Robot Initialization Done!");
        telemetry.update();
    }

    /**
     * Returns the robot's Y position from odometry.
     * Uses the follower created during initControls().
     * Returns NaN if follower is not yet initialized.
     */
    public double getRobotY() {
        return (follower != null) ? follower.getPose().getY() : Double.NaN;
    }

    /**
     * Helper method to determine if this OpMode is running in autonomous mode.
     * Checks for @Autonomous annotation on the class.
     */
    public boolean isAutonomousMode() {
        return this.getClass().isAnnotationPresent(com.qualcomm.robotcore.eventloop.opmode.Autonomous.class);
    }

    public DcMotor initializeMotor(String name) {
         /*This is just a handy dandy function which saves a few lines and looks cool,
         it initializes the motor and it also initializers the motor power logs for this motor*/
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        /*
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag)
                ;
         */

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

    public void displayRpmTelemetry() {
        telemetry.addData("Actual ShotGun RPM", ejectionMotor.getVelocity() * 60 / TICKS_PER_ROTATION); // convert from ticks per second to RPM
        telemetry.addData("ejectionMotor power", ejectionMotor.getPower());
        telemetry.addData("Actual ShotGun TPS", ejectionMotor.getVelocity()); // convert from ticks per second to RPM
        telemetry.addData("Shooting Power Mode", shootingPowerMode.toString());
    }

}
