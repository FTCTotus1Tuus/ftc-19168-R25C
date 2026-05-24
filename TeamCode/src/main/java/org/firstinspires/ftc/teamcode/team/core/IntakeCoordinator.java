package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team.fsm.IntakeFSM;

/**
 * Coordinates intake-related control mapping and lifecycle updates.
 */
public class IntakeCoordinator {

    private final IntakeFSM intakeFSM;

    public IntakeCoordinator(IntakeFSM intakeFSM) {
        this.intakeFSM = intakeFSM;
    }

    public void updateActiveIntake(double currentTime, Telemetry telemetry) {
        if (intakeFSM.getState() == IntakeFSM.States.INTAKING) {
            intakeFSM.readSensors(currentTime, telemetry);
            intakeFSM.update(currentTime, telemetry);
            intakeFSM.writeOutputs(telemetry);
        }
    }

    public void handleDriverControls(boolean intakeRequested, boolean ejectRequested, boolean offRequested) {
        if (intakeRequested) {
            intakeFSM.startIntaking();
        } else if (ejectRequested) {
            intakeFSM.reverse();
        } else if (offRequested) {
            intakeFSM.off();
        }
    }
}

