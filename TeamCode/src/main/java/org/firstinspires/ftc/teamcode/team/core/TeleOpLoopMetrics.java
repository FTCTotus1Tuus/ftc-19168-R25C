package org.firstinspires.ftc.teamcode.team.core;

/**
 * Immutable per-iteration runtime metrics collected from the OpMode shell.
 *
 * This keeps loop coordinator signatures short and gives names to values that
 * would otherwise be passed around as unlabeled doubles.
 */
public class TeleOpLoopMetrics {
    public final double currentTime;
    public final double ejectionMotorRpm;
    public final double ejectionMotorPower;
    public final double ejectionMotorVelocity;

    public TeleOpLoopMetrics(
            double currentTime,
            double ejectionMotorRpm,
            double ejectionMotorPower,
            double ejectionMotorVelocity
    ) {
        this.currentTime = currentTime;
        this.ejectionMotorRpm = ejectionMotorRpm;
        this.ejectionMotorPower = ejectionMotorPower;
        this.ejectionMotorVelocity = ejectionMotorVelocity;
    }
}

