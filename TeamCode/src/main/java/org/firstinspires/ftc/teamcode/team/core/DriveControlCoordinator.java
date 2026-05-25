package org.firstinspires.ftc.teamcode.team.core;

import com.pedropathing.follower.Follower;

/**
 * Coordinates TeleOp drive stick shaping and follower drive command emission.
 */
public class DriveControlCoordinator {

    public void applyTeleOpDrive(
            boolean isAutoParking,
            double leftStickY,
            double leftStickX,
            double rightStickX,
            double deadzone,
            double inputExponent,
            double speedScale,
            double speedScaleTurn,
            double rotationScale,
            Follower follower
    ) {
        if (isAutoParking) {
            // Auto-park owns drive commands while active.
            return;
        }

        // Deadzone first, then invert to keep forward stick as positive forward command.
        double rawY = (Math.abs(leftStickY) <= deadzone) ? 0 : -leftStickY;
        double rawX = (Math.abs(leftStickX) <= deadzone) ? 0 : -leftStickX;
        double rawR = (Math.abs(rightStickX) <= deadzone) ? 0 : -rightStickX;

        // Exponential shaping gives finer low-speed control while preserving full-range output.
        double shapedY = Math.signum(rawY) * Math.pow(Math.abs(rawY), inputExponent);
        double shapedX = Math.signum(rawX) * Math.pow(Math.abs(rawX), inputExponent);
        double shapedR = Math.signum(rawR) * Math.pow(Math.abs(rawR), inputExponent);

        // Keep existing behavior: reduce forward when rotational input is active.
        if (Math.abs(rightStickX) > deadzone) {
            follower.setTeleOpDrive(shapedY * speedScaleTurn, shapedX * speedScale, shapedR * rotationScale, true);
        } else {
            double forward = shapedY * speedScale;
            double strafe = shapedX * speedScale;
            double turn = shapedR * rotationScale;
            follower.setTeleOpDrive(forward, strafe, turn, true);
        }
    }
}


