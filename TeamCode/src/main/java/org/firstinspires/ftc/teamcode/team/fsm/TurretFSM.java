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
    private Servo turretServo;

    // HARDWARE FIXED CONSTANTS
    public static final int FIVE_ROTATION_SERVO_SPAN_DEG = 1800; // Degrees of rotation (5-rotation goBILDA servo)

    /**
     * Turret gear reduction ratio (input:output).
     *
     * A gear reduction of N:1 means the servo (input) must turn N times for the
     * turret (output) to turn once.  Equivalently, one full servo sweep of 1800°
     * produces  1800 / N  degrees of turret rotation.
     *
     * History:
     *   6.0  – original code assumption (gave 1800/6 = 300 °/unit — turret overshot)
     *   5.0  – CAD-specified ratio       (gives 1800/5 = 360 °/unit)
     *   ~4.55 – tape-measured on robot   (gives 1800/4.55 ≈ 395.6 °/unit)
     *   5.8  – field-tuned 3/3/2026      (gives 1800/5.8 ≈ 310.3 °/unit — aims on target)
     *
     * This is a public static (not final) so it can be tuned live via FTC Dashboard.
     *
     * HOW TO TUNE:
     *   If the turret OVERSHOOTS (aims too far left when pointing left, too far
     *   right when pointing right), the real ratio is LOWER than the current value
     *   → DECREASE this number.
     *
     *   If the turret UNDERSHOOTS (doesn't turn far enough), the real ratio is
     *   HIGHER → INCREASE this number.
     */
    public static double TURRET_GEAR_RATIO = 5.8;

    /**
     * Returns the effective turret degrees of rotation per full servo unit (0.0→1.0).
     * Recomputed from the live TURRET_GEAR_RATIO every call so FTC Dashboard
     * changes take effect immediately.
     * <p>
     * Example values:
     * ratio 6.0  → 1800/6    = 300.0 °/unit  (old, wrong)
     * ratio 5.8  → 1800/5.8  ≈ 310.3 °/unit  (current, field-tuned)
     * ratio 5.0  → 1800/5    = 360.0 °/unit  (CAD)
     * ratio 4.55 → 1800/4.55 ≈ 395.6 °/unit  (tape)
     */
    private static double turretDegPerServoUnit() {
        return (double) FIVE_ROTATION_SERVO_SPAN_DEG / TURRET_GEAR_RATIO;
    }

    // HARDWARE TUNING CONSTANTS
    public static double TURRET_ROTATION_INCREMENT = 0.001;
    public static double TURRET_ROTATION_INCREMENT_FAST = 0.003;
    public static double TURRET_POSITION_CENTER = 0.5;
    public static double TURRET_OFFSET_DEG_RED = 1.0;
    public static double TURRET_OFFSET_DEG_BLUE = -3.0;
    /**
     * Distance (inches) from the robot's odometry center of rotation to the turret pivot,
     * measured along the robot's forward axis. Negative = toward the rear of the robot.
     * Corrects the parallax aiming error that grows as the robot turns away from the goal.
     */
    public static double TURRET_PIVOT_OFFSET_INCHES = -1.5;

    /**
     * Distance (inches) from the robot's odometry center of rotation to the turret pivot,
     * measured perpendicular to the robot's forward axis (i.e. sideways).
     * Positive = turret pivot is to the LEFT of center, Negative = to the RIGHT.
     * Corrects a lateral parallax error that causes heading-dependent aiming drift
     * (the turret aims too far right at some headings and too far left at others).
     */
    public static double TURRET_PIVOT_OFFSET_LATERAL_INCHES = 0;

    // Turret range of motion in turret degrees (physically measurable on the robot).
    // Positive = CCW from center (left), Negative = CW from center (right).
    // Change these when hardware changes; servo clamp limits are derived automatically.
    public static double TURRET_MAX_DEG_LEFT = 150.0;  // degrees CCW from center
    public static double TURRET_MAX_DEG_RIGHT = 180.0;  // degrees CW  from center

    // DYNAMIC VARIABLES
    private double TURRET_SERVO_POSITION_MAX_LEFT = TURRET_POSITION_CENTER + 0.1; // Increases counter clockwise
    private double TURRET_SERVO_POSITION_MAX_RIGHT = TURRET_POSITION_CENTER - 0.1; // Decreases clockwise

    private double currentTurretPosition;

    // Alliance flag — determines which static offset constant is read live each loop.
    // Reading the static directly (not copying it) means Dashboard changes take effect immediately.
    private boolean isBlueAlliance = false;


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
    public double getTurretHeading() {
        // (servo - center) * degreesPerUnit gives turret degrees offset from forward
        return (currentTurretPosition - TURRET_POSITION_CENTER)
                * turretDegPerServoUnit();
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
        // Shift origin from robot center-of-rotation to the turret pivot.
        //
        // The turret pivot may be offset from the odometry center in TWO directions:
        //   TURRET_PIVOT_OFFSET_INCHES         — forward/backward along the robot's heading axis
        //                                        (negative = toward the rear of the robot)
        //   TURRET_PIVOT_OFFSET_LATERAL_INCHES — left/right perpendicular to the heading axis
        //                                        (positive = left of robot center, negative = right)
        //
        // To convert these robot-frame offsets into field coordinates, we project each
        // offset along its corresponding field-frame unit vector:
        //   Forward axis  in field frame:  ( cos(heading),  sin(heading) )
        //   Left    axis  in field frame:  ( -sin(heading), cos(heading) )   ← 90° CCW from forward
        //
        // Combined:
        //   pivotX = robotX + cos(h) * forward + (-sin(h)) * lateral
        //   pivotY = robotY + sin(h) * forward + cos(h)    * lateral
        double pivotX = robotX
                + Math.cos(robotHeadingRadians) * TURRET_PIVOT_OFFSET_INCHES
                - Math.sin(robotHeadingRadians) * TURRET_PIVOT_OFFSET_LATERAL_INCHES;
        double pivotY = robotY
                + Math.sin(robotHeadingRadians) * TURRET_PIVOT_OFFSET_INCHES
                + Math.cos(robotHeadingRadians) * TURRET_PIVOT_OFFSET_LATERAL_INCHES;

        // Calculate vector from turret pivot to goal
        double deltaX = goalX - pivotX;
        double deltaY = goalY - pivotY;

        // Calculate bearing angle to goal in radians (0 = along X axis, counterclockwise positive)
        double bearingRobotToGoalRadians = Math.atan2(deltaY, deltaX);

        // Calculate the angle relative to robot heading
        // Positive = counterclockwise from robot forward = left
        double angleTurretRelativeToRobotRadians = bearingRobotToGoalRadians - robotHeadingRadians;

        // Normalize to [-π, π] range (pure wrapping only — no offset applied here)
        while (angleTurretRelativeToRobotRadians > Math.PI)
            angleTurretRelativeToRobotRadians -= 2 * Math.PI;
        while (angleTurretRelativeToRobotRadians < -Math.PI)
            angleTurretRelativeToRobotRadians += 2 * Math.PI;

        // Apply fine mechanical trim AFTER normalization.
        // Read the static constant directly so FTC Dashboard changes take effect immediately.
        double offsetDeg = isBlueAlliance ? TURRET_OFFSET_DEG_BLUE : TURRET_OFFSET_DEG_RED;
        angleTurretRelativeToRobotRadians += Math.toRadians(offsetDeg);

        // Convert to degrees
        return Math.toDegrees(angleTurretRelativeToRobotRadians);
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
        // Convert turret angle to servo position using the tunable gear ratio.
        // servo = center + (turretDegrees / turretDegPerServoUnit())
        //
        // With ratio 5.8 (field-tuned):  turretDegPerServoUnit ≈ 310.3
        //   30° left  → 0.5 + 30/310.3 ≈ 0.597
        //   45° left  → 0.5 + 45/310.3 ≈ 0.645
        //   20° right → 0.5 − 20/310.3 ≈ 0.436
        double servoPosition = TURRET_POSITION_CENTER +
                (angleDegrees / turretDegPerServoUnit());

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
        // Convert a field-relative bearing (degrees) to a servo position.
        // Same math as calculateServoPositionFromAngle but without clamping.
        double targetServoPos = TURRET_POSITION_CENTER + bearingDeg / turretDegPerServoUnit();
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
        rotateLeft(TURRET_ROTATION_INCREMENT);
    }

    public void rotateLeftFast() {
        rotateLeft(TURRET_ROTATION_INCREMENT_FAST);
    }

    public void rotateLeft(double rotationIncrement) {
        //updating the current turret position to be in range of the min and max
        double clampedTargetPosition = clampT(currentTurretPosition + rotationIncrement, TURRET_SERVO_POSITION_MAX_LEFT, TURRET_SERVO_POSITION_MAX_RIGHT);
        //sets turret position
        this.setPosition(clampedTargetPosition);
    }

    public void rotateRight() {
        rotateRight(TURRET_ROTATION_INCREMENT);
    }

    public void rotateRightFast() {
        rotateRight(TURRET_ROTATION_INCREMENT_FAST);
    }

    public void rotateRight(double rotationIncrement) {
        //updating the current turret position to be in range of the min and max
        double clampedTargetPosition = clampT(currentTurretPosition - rotationIncrement, TURRET_SERVO_POSITION_MAX_LEFT, TURRET_SERVO_POSITION_MAX_RIGHT);
        //sets turret position
        this.setPosition(clampedTargetPosition);
    }

    public void center() {
        this.setPosition(TURRET_POSITION_CENTER); // set to center position
    }

    public void setOffsetBlue() {
        this.isBlueAlliance = true;
    }

    public void setOffsetRed() {
        this.isBlueAlliance = false;
    }

    /**
     * Servo clamp limits derived from degree limits and the current gear ratio.
     * Formula: servo = center ± (degrees / turretDegPerServoUnit())
     * Recomputed every loop so FTC Dashboard changes to TURRET_GEAR_RATIO,
     * TURRET_POSITION_CENTER, or the degree limits take effect immediately.
     */
    public void updateTurretServoLimits() {
        double degPerUnit = turretDegPerServoUnit();
        TURRET_SERVO_POSITION_MAX_LEFT = TURRET_POSITION_CENTER + TURRET_MAX_DEG_LEFT / degPerUnit;
        TURRET_SERVO_POSITION_MAX_RIGHT = TURRET_POSITION_CENTER - TURRET_MAX_DEG_RIGHT / degPerUnit;
    }

    public void update() {
        updateTurretServoLimits();
    }

}


