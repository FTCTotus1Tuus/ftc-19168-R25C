package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretFSM {
    private final DarienOpModeFSM opMode;

    /**
     * Constructor for TurretFSM
     * @param opMode The main op mode instance
     */
    public TurretFSM(DarienOpModeFSM opMode) {
        this.opMode = opMode;
    }

    /**
     * Effective turret degrees of rotation per full servo unit (0.0 to 1.0).
     * The 5-rotation servo spans 1800 servo-degrees across 0..1.
     * The 6:1 turret gear reduces that to 1800/6 = 300 turret-degrees per servo unit.
     */
    private static final double TURRET_DEG_PER_SERVO_UNIT =
            (double) DarienOpModeFSM.FIVE_ROTATION_SERVO_SPAN_DEG / DarienOpModeFSM.RATIO_BETWEEN_TURRET_GEARS; // = 300

    /**
     * Get the current turret heading in degrees based on the servo position.
     * Zero heading is straight ahead (forward motion of the robot).
     * Positive degrees = left, negative degrees = right.
     * @return The turret heading in degrees.
     */
    public double getTurretHeading(){
        // (servo - center) * 300 deg/unit gives turret degrees offset from forward
        return (opMode.currentTurretPosition - DarienOpModeFSM.TURRET_POSITION_CENTER)
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
    public double calculateTurretAngleFromOdometry(double goalX, double goalY,
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
    public double calculateServoPositionFromAngle(double angleDegrees) {
        // Convert turret angle to servo position.
        // TURRET_DEG_PER_SERVO_UNIT = 300 deg/unit (1800 servo-degrees / 6:1 gear ratio).
        // So: servo = center + (turretDegrees / 300)
        // Example: 30 deg left  → 0.5 + 30/300 = 0.60  (within left clamp 0.63) ✓
        //          45 deg left  → 0.5 + 45/300 = 0.65  → clamped to 0.63        ✓
        //          20 deg right → 0.5 - 20/300 = 0.433 (within right clamp 0.35) ✓
        double servoPosition = DarienOpModeFSM.TURRET_POSITION_CENTER +
                (angleDegrees / TURRET_DEG_PER_SERVO_UNIT);

        // Clamp to servo physical limits to prevent damage
        servoPosition = Math.max(opMode.TURRET_ROTATION_MAX_RIGHT,
                                 Math.min(opMode.TURRET_ROTATION_MAX_LEFT, servoPosition));

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
    public double calculateServoPositionFromOdometry(double goalX, double goalY,
                                                     double robotX, double robotY,
                                                     double robotHeadingRadians) {
        // Calculate the target turret angle based on (a) the robot position and heading and (b) the target goal
        double angleDegrees = calculateTurretAngleFromOdometry(goalX, goalY, robotX, robotY, robotHeadingRadians);
        // Convert the desired angle to a servo position
        return calculateServoPositionFromAngle(angleDegrees);
    }

}


