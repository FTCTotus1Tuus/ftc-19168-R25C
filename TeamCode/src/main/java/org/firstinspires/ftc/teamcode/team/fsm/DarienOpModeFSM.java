package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.team.core.RobotContainer;
import org.firstinspires.ftc.teamcode.team.config.DriveConfig;
import org.firstinspires.ftc.teamcode.team.MotorHelper;
import org.firstinspires.ftc.teamcode.team.services.AprilTagService;
import org.firstinspires.ftc.teamcode.team.services.AprilTagVisionService;
import org.firstinspires.ftc.teamcode.team.services.LocalizationService;
import org.firstinspires.ftc.teamcode.team.services.PreferencesService;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

/**
 * Base OpMode for Pedro pathing and state machine logic.
 * Extend this class for autonomous OpModes using Pedro pathing.
 */
@Config
@Configurable
public abstract class DarienOpModeFSM extends LinearOpMode {

    // Pedro pathing/state machine FSMs (declare as needed)
    // public PathFollowerFSM pathFollowerFSM;
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
    // Fixed encoder geometry constants.
    // Note: tunable drivetrain/scoring values now live under team.config.*

    public int targetGoalId = 0;
    protected RobotContainer robotContainer;
    protected AprilTagService aprilTagService;
    protected AprilTagVisionService visionService;
    protected LocalizationService localizationService;
    protected PreferencesService preferencesService;

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

        // Delegate robot hardware/services/subsystems initialization to container.
        if (robotContainer == null) {
            robotContainer = new RobotContainer(this);
        }
        robotContainer.initialize();
        visionService = robotContainer.getVisionService();
        aprilTagService = robotContainer.getAprilTagService();
        localizationService = robotContainer.getLocalizationService();
        preferencesService = robotContainer.getPreferencesService();

        ejectionMotor = robotContainer.getHardware().ejectionMotor;
        aprilTag = robotContainer.getAprilTag();
        visionPortal = robotContainer.getVisionPortal();
        MotorHelper = robotContainer.getMotorHelper();
        shootArtifactFSM = robotContainer.getShootArtifactFSM();
        shootPatternFSM = robotContainer.getShootPatternFSM();
        shotgunFSM = robotContainer.getShotgunFSM();
        turretFSM = robotContainer.getTurretFSM();
        gateFSM = robotContainer.getGateFSM();
        intakeFSM = robotContainer.getIntakeFSM();
        shootingFSM = robotContainer.getShootingFSM();
        follower = robotContainer.getFollower();

        // Set default shooting power mode: ODOMETRY for Autos, MANUAL for TeleOp
        shootingPowerMode = isAutonomousMode() ? ShootingPowerModes.ODOMETRY : ShootingPowerModes.MANUAL;


        telemetry.addLine("FTC 19168 Robot Initialization Done!");

        // Camera validation telemetry — shows whether portal built and exposure applied
        if (visionService != null) {
            telemetry.addData("Camera Health", visionService.getHealth().toString());
            telemetry.addData("Camera Detail", visionService.getHealthDetail());
        }

        telemetry.update();
    }

    /**
     * Cleanly tears down the camera portal and any other resources.
     * Call at the end of runOpMode() (after the main loop) so the camera is
     * properly closed before the next OpMode starts.
     */
    public void stopRobot() {
        if (robotContainer != null) {
            robotContainer.close();
        }
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

    /** Read this camera's minimum and maximum Exposure and Gain settings. */
    public int[] getCameraSettings() {
        return (visionService != null) ? visionService.getCameraSettings() : new int[]{0, 0, 0, 0};
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
            double ticksPerSecond = (targetRPM / 60.0) * DriveConfig.TICKS_PER_ROTATION;
            return ticksPerSecond;
        }
        catch (Exception e) {
            // telemetry.addData("Ticks/Sec Adjustment Error", e.getMessage());
            return requestedRPM; // if error, return requested power unmodified
        }

    }

    public double getRpmFromTicksPerSecond(double ticksPerSecond) {
        try {
            double rpm = (ticksPerSecond * 60.0) / DriveConfig.TICKS_PER_ROTATION;
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
        telemetry.addData("Actual ShotGun RPM", ejectionMotor.getVelocity() * 60 / DriveConfig.TICKS_PER_ROTATION); // convert from ticks per second to RPM
        telemetry.addData("ejectionMotor power", ejectionMotor.getPower());
        telemetry.addData("Actual ShotGun TPS", ejectionMotor.getVelocity()); // convert from ticks per second to RPM
        telemetry.addData("Shooting Power Mode", shootingPowerMode.toString());
    }

    /**
     * Adds a shared trace telemetry schema for TeleOp and autonomous loops.
     */
    protected void addTraceTelemetry(String phase, String state, double stateTimerSec) {
        telemetry.addData("TRACE/Phase", phase);
        telemetry.addData("TRACE/State", state);
        telemetry.addData("TRACE/RuntimeSec", String.format("%.2f", getRuntime()));
        telemetry.addData("TRACE/StateTimerSec", String.format("%.2f", stateTimerSec));

        if (follower != null) {
            telemetry.addData(
                    "TRACE/Pose",
                    String.format(
                            "x=%.1f y=%.1f h=%.1fdeg",
                            follower.getPose().getX(),
                            follower.getPose().getY(),
                            Math.toDegrees(follower.getPose().getHeading())
                    )
            );
        } else {
            telemetry.addData("TRACE/Pose", "unavailable");
        }

        telemetry.addData("TRACE/TurretMode", turretFSM != null ? turretFSM.getState() : "unavailable");
        telemetry.addData("TRACE/ShooterMode", shootingPowerMode);
        telemetry.addData("TRACE/ShooterStage", shootingFSM != null ? shootingFSM.getStage() : "unavailable");
    }

}
