package org.firstinspires.ftc.teamcode.team.core;

/**
 * Immutable one-iteration input bundle for the TeleOp loop coordinator.
 */
public class TeleOpLoopIterationInput {
    public final DriverOneBindings driverOne;
    public final DriverTwoBindings driverTwo;
    public final TeleOpLoopMetrics metrics;

    public TeleOpLoopIterationInput(
            DriverOneBindings driverOne,
            DriverTwoBindings driverTwo,
            TeleOpLoopMetrics metrics
    ) {
        this.driverOne = driverOne;
        this.driverTwo = driverTwo;
        this.metrics = metrics;
    }
}

