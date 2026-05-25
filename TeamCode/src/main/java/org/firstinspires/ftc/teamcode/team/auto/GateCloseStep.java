package org.firstinspires.ftc.teamcode.team.auto;

import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;

/**
 * Step to close the gate (prevents shooting while traveling).
 * Returns isDone() immediately.
 */
public class GateCloseStep implements AutoStep {

    @Override
    public void init(DarienOpModeFSM opMode) {
        opMode.gateFSM.close();
    }

    @Override
    public void update(DarienOpModeFSM opMode, double elapsedSec) {
        // Nothing to do
    }

    @Override
    public boolean isDone() {
        return true;
    }

    @Override
    public void cleanup(DarienOpModeFSM opMode) {
        // Already handled in init()
    }

    @Override
    public String getName() {
        return "GateClose";
    }
}

