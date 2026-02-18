package org.firstinspires.ftc.teamcode.team.fsm;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoIncrementalFSM {
    private Servo servo;
    private double startPos, endPos, duration;
    private double startTime;
    private boolean running = false;

    /**
     * Constructor for ServoIncrementalFSM.
     *
     * @param servo The servo to be controlled.
     */
    public ServoIncrementalFSM(Servo servo) {
        this.servo = servo;
    }

    /**
     * Start moving the servo from startPos to endPos over the specified duration.
     *
     * @param endPos      The target servo position (0.0 to 1.0).
     * @param startPos    The starting servo position (0.0 to 1.0).
     * @param duration    The duration over which to move the servo, in seconds.
     * @param currentTime The current time in seconds, usually from opMode.getRuntime().
     */
    public void start(double endPos, double startPos, double duration, double currentTime) {
        this.startPos = startPos;
        this.endPos = endPos;
        this.duration = duration;
        this.startTime = currentTime;
        this.running = true;
    }

    /**
     * Call this method periodically to update the servo position.
     *
     * @param currentTime The current time in seconds, usually from opMode.getRuntime().
     */
    public void update(double currentTime) {
        if (!running) return;
        double elapsed = currentTime - startTime;
        if (elapsed >= duration) {
            servo.setPosition(endPos);
            running = false;
            return;
        }
        // 1.0 represents the maximum progress as a fraction (100%) of the movement.
        // The progress value ranges from 0.0 (start) to 1.0 (complete), regardless
        // of the duration in seconds. This ensures the servo position calculation
        // does not exceed the intended range.
        double progress = Math.min(1.0, (currentTime - startTime) / duration);
        // The second line calculates the new position of the servo based on the progress.
        // It uses linear interpolation to determine the position between startPos and endPos.
        // The formula startPos + (endPos - startPos) * progress ensures that the position
        // smoothly transitions from the starting value to the ending value as the progress increases
        double newPos = startPos + (endPos - startPos) * progress;
        servo.setPosition(newPos);
    }

    /**
     * Check if the servo is currently moving.
     *
     * @return True if the servo is moving, false otherwise.
     */
    public boolean isRunning() {
        return running;
    }

    public double getTargetPosition() {
        return endPos;
    }
}
