package org.firstinspires.ftc.teamcode.team.core;

/**
 * Immutable snapshot of driver one controls for a single TeleOp loop.
 */
public class DriverOneBindings {

    public final double driveForwardAxis;
    public final double driveStrafeAxis;
    public final double driveTurnAxis;

    public final boolean intakeRequested;
    public final boolean ejectRequested;
    public final boolean intakeOffRequested;

    public final boolean autoParkRequested;
    public final boolean odometryResetRequested;

    public DriverOneBindings(
            double driveForwardAxis,
            double driveStrafeAxis,
            double driveTurnAxis,
            boolean intakeRequested,
            boolean ejectRequested,
            boolean intakeOffRequested,
            boolean autoParkRequested,
            boolean odometryResetRequested
    ) {
        this.driveForwardAxis = driveForwardAxis;
        this.driveStrafeAxis = driveStrafeAxis;
        this.driveTurnAxis = driveTurnAxis;
        this.intakeRequested = intakeRequested;
        this.ejectRequested = ejectRequested;
        this.intakeOffRequested = intakeOffRequested;
        this.autoParkRequested = autoParkRequested;
        this.odometryResetRequested = odometryResetRequested;
    }
}

