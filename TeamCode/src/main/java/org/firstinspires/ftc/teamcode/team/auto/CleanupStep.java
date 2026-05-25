package org.firstinspires.ftc.teamcode.team.auto;

import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;

/**
 * Final step that shuts down all motors and systems after auto sequence completes.
 * Completes immediately on first update.
 */
public class CleanupStep implements AutoStep {

    private boolean isInitialized;

    public CleanupStep() {
        this.isInitialized = false;
    }

    @Override
    public void init(DarienOpModeFSM opMode) {
        opMode.shotgunFSM.toOff();
        opMode.intakeFSM.off();
        opMode.gateFSM.close();
        isInitialized = true;
    }

    @Override
    public void update(DarienOpModeFSM opMode, double elapsedSec) {
        // Nothing to do; cleanup is complete
    }

    @Override
    public boolean isDone() {
        return isInitialized;
    }

    @Override
    public void cleanup(DarienOpModeFSM opMode) {
        // Already cleaned up in init()
    }

    @Override
    public String getName() {
        return "Cleanup";
    }
}

