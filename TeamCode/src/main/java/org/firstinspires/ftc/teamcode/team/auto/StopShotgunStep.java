package org.firstinspires.ftc.teamcode.team.auto;

import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;

/**
 * Step to stop the shotgun motor.
 * Returns isDone() immediately; use at the end of the autonomous sequence.
 */
public class StopShotgunStep implements AutoStep {

    @Override
    public void init(DarienOpModeFSM opMode) {
        opMode.shotgunFSM.toOff();
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
        // Already stopped in init()
    }

    @Override
    public String getName() {
        return "StopShotgun";
    }
}

