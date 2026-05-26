package org.firstinspires.ftc.teamcode.team.auto;

import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;
import org.firstinspires.ftc.teamcode.team.config.ShooterConfig;

/**
 * Step to start shotgun spinning at autonomous RPM.
 * Returns isDone() immediately, allowing the plan to continue.
 * The shotgun remains spinning until stopped by StopShotgunStep or plan end.
 */
public class ShotgunSpinStep implements AutoStep {

    @Override
    public void init(DarienOpModeFSM opMode) {
        opMode.shotgunFSM.toPowerUp(ShooterConfig.SHOT_GUN_POWER_UP_RPM_AUTO);
    }

    @Override
    public void update(DarienOpModeFSM opMode, double elapsedSec) {
        // Shotgun should be kept at RPM via a background task in the main loop,
        // not here. This step is just a marker for when to start.
    }

    @Override
    public boolean isDone() {
        return true;  // Completes immediately to allow next step
    }

    @Override
    public void cleanup(DarienOpModeFSM opMode) {
        // Don't stop shotgun here; it continues spinning
    }

    @Override
    public String getName() {
        return "StartShotgun";
    }
}


