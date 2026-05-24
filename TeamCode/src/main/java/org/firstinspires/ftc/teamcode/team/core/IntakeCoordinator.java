package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team.fsm.IntakeFSM;
import org.firstinspires.ftc.teamcode.team.subsystems.IntakeControl;

/**
 * Coordinates intake-related control mapping and lifecycle updates.
 */
public class IntakeCoordinator {

    private final IntakeControl intakeControl;
    private final IntakeFSM intakeFSM;

    public IntakeCoordinator(IntakeControl intakeControl, IntakeFSM intakeFSM) {
        this.intakeControl = intakeControl;
        this.intakeFSM = intakeFSM;
    }

    public void updateActiveIntake(double currentTime, Telemetry telemetry) {
        if (intakeControl.isIntaking()) {
            intakeFSM.readSensors(currentTime, telemetry);
            intakeFSM.update(currentTime, telemetry);
            intakeFSM.writeOutputs(telemetry);
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

