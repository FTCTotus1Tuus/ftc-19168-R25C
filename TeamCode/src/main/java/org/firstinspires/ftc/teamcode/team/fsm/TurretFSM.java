package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class TurretFSM {

    public enum TurretStates {MANUAL, CAMERA, ODOMETRY}

    private TurretStates turretState = TurretStates.MANUAL;

    private final HardwareMap hardwareMap;

    // HARDWARE DEVICES
    public Servo turretServo;

    // HARDWARE FIXED CONSTANTS
    public static final int FIVE_ROTATION_SERVO_SPAN_DEG = 1800; // Degrees of rotation (5-rotation goBILDA servo)
    public static final int RATIO_BETWEEN_TURRET_GEARS = 6;

    /**
     * Effective turret degrees of rotation per full servo unit (0.0 to 1.0).
     * The 5-rotation servo spans 1800 servo-degrees across 0..1.
     * The 6:1 turret gear reduces that to 1800/6 = 300 turret-degrees per servo unit.
     */
    private static final double TURRET_DEG_PER_SERVO_UNIT =
            (double) FIVE_ROTATION_SERVO_SPAN_DEG / RATIO_BETWEEN_TURRET_GEARS; // = 300

    // HARDWARE TUNING CONSTANTS
    public static double TURRET_ROTATION_INCREMENT = 0.004;
    public static double TURRET_POSITION_CENTER = 0.5;
    public static double TURRET_OFFSET_RED = 0.01;
    public static double TURRET_OFFSET_BLUE = 0.01;


    // Turret range of motion in turret degrees (physically measurable on the robot).
    // Positive = CCW from center (left), Negative = CW from center (right).
    // Change these when hardware changes; servo clamp limits are derived automatically.
    public static double TURRET_MAX_DEG_LEFT = 90.0;  // degrees CCW from center
    public static double TURRET_MAX_DEG_RIGHT = 90.0;  // degrees CW  from center

    // DYNAMIC VARIABLES
    private double TURRET_SERVO_POSITION_MAX_LEFT = TURRET_POSITION_CENTER + 0.1; // Increases counter clockwise
    private double TURRET_SERVO_POSITION_MAX_RIGHT = TURRET_POSITION_CENTER - 0.1; // Decreases clockwise

    private double currentTurretPosition;
    private double turretOffset = 0;


    /**
     * Constructor for TurretFSM
     * @param hardwareMap The main op mode's hardware map
     */
    public TurretFSM(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        turretServo = this.hardwareMap.get(Servo.class, "turretServo");
    }

    /**
     * Initializes turret by defaulting to center position.
     */
    public void init() {
        updateTurretServoLimits();
    }

    /**
     * Get the current turret heading in degrees based on the servo position.
     * Zero heading is straight ahead (forward motion of the robot).
     * Positive degrees = left, negative degrees = right.
     * @return The turret heading in degrees.
     */
    public double getTurretHeading(){
        // (servo - center) * 300 deg/unit gives turret degrees offset from forward
        return (currentTurretPosition - TURRET_POSITION_CENTER)
                * TURRET_DEG_PER_SERVO_UNIT;
    }

    /**
     * Calculate the turret aiming angle based on robot odometry and goal position.
     * The angle is relative to the robot's forward direction (0 degrees = straight ahead).
     *
     * @param goalX               Goal X position in inches (field coordinates)
     * @param goalY               Goal Y position in inches (field coordinates)
     * @param robotX              Robot X position in inches (field coordinates)
     * @param robotY              Robot Y position in inches (field coordinates)
     * @param robotHeadingRadians Robot heading in radians (0 = forward along X axis)
     * @return Turret angle in degrees relative to robot forward (positive = left, negative = right)
     */
    private double calculateTurretAngleFromOdometry(double goalX, double goalY,
                                                   double robotX, double robotY,
                                                   double robotHeadingRadians) {
        // Calculate vector from robot to goal
        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;

        // Calculate bearing angle to goal in radians (0 = along X axis, counterclockwise positive)
        double bearingToGoalRadians = Math.atan2(deltaY, deltaX);

        // Calculate the angle relative to robot heading
        // Positive = counterclockwise from robot forward = left
        double relativeAngleRadians = bearingToGoalRadians - robotHeadingRadians;

        // Normalize to [-π, π] range
        while (relativeAngleRadians > Math.PI) {
            relativeAngleRadians -= 2 * Math.PI;
        }
        while (relativeAngleRadians < -Math.PI) {
            relativeAngleRadians += 2 * Math.PI;
        }

        // Convert to degrees
        return Math.toDegrees(relativeAngleRadians);
    }

    /**
     * Convert a turret angle (in degrees) to a servo position, clamped to safe limits.
     * Zero angle = straight ahead (center position).
     * Positive angles = left, Negative angles = right.
     *
     * @param angleDegrees Turret angle in degrees
     * @return Servo position (0.0 to 1.0), clamped to [TURRET_ROTATION_MAX_RIGHT, TURRET_ROTATION_MAX_LEFT]
     */
    private double calculateServoPositionFromAngle(double angleDegrees) {
        // Convert turret angle to servo position.
        // TURRET_DEG_PER_SERVO_UNIT = 300 deg/unit (1800 servo-degrees / 6:1 gear ratio).
        // So: servo = center + (turretDegrees / 300)
        // Example: 30 deg left  → 0.5 + 30/300 = 0.60  (within left clamp 0.63) ✓
        //          45 deg left  → 0.5 + 45/300 = 0.65  → clamped to 0.63        ✓
        //          20 deg right → 0.5 - 20/300 = 0.433 (within right clamp 0.35) ✓
        double servoPosition = TURRET_POSITION_CENTER + turretOffset +
                (angleDegrees / TURRET_DEG_PER_SERVO_UNIT);

        // Clamp to servo physical limits to prevent damage
        servoPosition = Math.max(TURRET_SERVO_POSITION_MAX_RIGHT,
                                 Math.min(TURRET_SERVO_POSITION_MAX_LEFT, servoPosition));

        return servoPosition;
    }

    /**
     * Calculate turret servo position directly from robot and goal positions.
     * Convenience method that combines odometry calculation with servo positioning.
     *
     * @param goalX               Goal X position in inches
     * @param goalY               Goal Y position in inches
     * @param robotX              Robot X position in inches
     * @param robotY              Robot Y position in inches
     * @param robotHeadingRadians Robot heading in radians
     * @return Servo position (0.0 to 1.0), clamped to safe limits
     */
    private double calculateServoPositionFromOdometry(double goalX, double goalY,
                                                     double robotX, double robotY,
                                                     double robotHeadingRadians) {
        // Calculate the target turret angle based on (a) the robot position and heading and (b) the target goal
        double angleDegrees = calculateTurretAngleFromOdometry(goalX, goalY, robotX, robotY, robotHeadingRadians);
        // Convert the desired angle to a servo position
        return calculateServoPositionFromAngle(angleDegrees);
    }

    /**
     * Set the turret position directly from robot and goal positions.
     *
     * @param goalX               Goal X position in inches
     * @param goalY               Goal Y position in inches
     * @param robotX              Robot X position in inches
     * @param robotY              Robot Y position in inches
     * @param robotHeadingRadians Robot heading in radians
     * @return Servo position (0.0 to 1.0), clamped to safe limits
     */
    public void setPositionFromOdometry(double goalX, double goalY,
                                        double robotX, double robotY,
                                        double robotHeadingRadians) {
        double targetServoPos = calculateServoPositionFromOdometry(goalX, goalY, robotX, robotY, robotHeadingRadians);
        setPosition(targetServoPos);
    }

    //private clamp test
    private static double clampT(double v, double min, double max) {
        if (Double.isNaN(v)) return min;               // defensive: treat NaN as min
        if (min > max) {                               // tolerate inverted bounds
            double t = min; min = max; max = t;
        }
        return Math.max(min, Math.min(max, v));
    }

    public void setPosition(double targetServoPos) {
        currentTurretPosition = targetServoPos;
        turretServo.setPosition(targetServoPos);
    }

    public double getPosition() {
        return currentTurretPosition;
    }

    public void alignToBearing(double bearingDeg) {
        double targetServoPos = TURRET_POSITION_CENTER + turretOffset + RATIO_BETWEEN_TURRET_GEARS * bearingDeg / FIVE_ROTATION_SERVO_SPAN_DEG;
        if (!Double.isNaN(targetServoPos)) {
            this.setPosition(targetServoPos);
        }
    }

    public void setState(TurretStates state) {
        turretState = state;
    }

    public TurretStates getState() {
        return turretState;
    }

    public void rotateLeft() {
        //updating the current turret position to be in range of the min and max
        double clampedTargetPosition = clampT(currentTurretPosition + TURRET_ROTATION_INCREMENT, TURRET_SERVO_POSITION_MAX_LEFT, TURRET_SERVO_POSITION_MAX_RIGHT);
        //sets turret position
        this.setPosition(clampedTargetPosition);
    }

    public void rotateRight() {
        //updating the current turret position to be in range of the min and max
        double clampedTargetPosition = clampT(currentTurretPosition - TURRET_ROTATION_INCREMENT, TURRET_SERVO_POSITION_MAX_LEFT, TURRET_SERVO_POSITION_MAX_RIGHT);
        //sets turret position
        this.setPosition(clampedTargetPosition);
    }

    public void center() {
        this.setPosition(TURRET_POSITION_CENTER); // set to center position
    }

    public void setOffsetBlue() {
        this.turretOffset = TURRET_OFFSET_BLUE;
    }

    public void setOffsetRed() {
        this.turretOffset = TURRET_OFFSET_RED;
    }

    /**
     * Servo clamp limits derived from degree limits.
     * Formula: servo = center ± (degrees / (FIVE_ROTATION_SERVO_SPAN_DEG / RATIO_BETWEEN_TURRET_GEARS))
     * = 0.5 ± (degrees / 300)
     * These are computed at runtime so they always stay in sync with the degree constants above.
     *
     */
    public void updateTurretServoLimits() {
        TURRET_SERVO_POSITION_MAX_LEFT = TURRET_POSITION_CENTER + TURRET_MAX_DEG_LEFT / ((double) FIVE_ROTATION_SERVO_SPAN_DEG / RATIO_BETWEEN_TURRET_GEARS);
        TURRET_SERVO_POSITION_MAX_RIGHT = TURRET_POSITION_CENTER - TURRET_MAX_DEG_RIGHT / ((double) FIVE_ROTATION_SERVO_SPAN_DEG / RATIO_BETWEEN_TURRET_GEARS);
    }

    public void update() {
        updateTurretServoLimits();
    }

}


