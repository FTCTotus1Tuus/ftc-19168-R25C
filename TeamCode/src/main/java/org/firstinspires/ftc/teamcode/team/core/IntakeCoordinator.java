package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team.subsystems.IntakeLifecycleControl;

/**
 * Coordinates intake-related control mapping and lifecycle updates.
 */
public class IntakeCoordinator {

    private final IntakeLifecycleControl intakeControl;

    public IntakeCoordinator(IntakeLifecycleControl intakeControl) {
        this.intakeControl = intakeControl;
    }

    public void updateActiveIntake(double currentTime, Telemetry telemetry) {
        if (intakeControl.isIntaking()) {
            // Single interface owns intake lifecycle order for deterministic updates.
            intakeControl.readSensors(currentTime, telemetry);
            intakeControl.update(currentTime, telemetry);
            intakeControl.writeOutputs(telemetry);
        }
    }

    public void handleDriverControls(boolean intakeRequested, boolean ejectRequested, boolean offRequested) {
        if (intakeRequested) {
            intakeControl.startIntaking();
        } else if (ejectRequested) {
            intakeControl.reverse();
        } else if (offRequested) {
            intakeControl.off();
        }
    }
}

