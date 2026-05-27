package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;

/**
 * Mutable-by-replacement state carried across TeleOp loop iterations.
 */
public class TeleOpLoopState {
    public final boolean isAutoParking;
    public final double autoParkStartTime;
    public final String autoAlliance;
    public final DarienOpModeFSM.ShootingPowerModes shootingPowerMode;
    public final DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch;

    public TeleOpLoopState(
            boolean isAutoParking,
            double autoParkStartTime,
            String autoAlliance,
            DarienOpModeFSM.ShootingPowerModes shootingPowerMode,
            DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch
    ) {
        this.isAutoParking = isAutoParking;
        this.autoParkStartTime = autoParkStartTime;
        this.autoAlliance = autoAlliance;
        this.shootingPowerMode = shootingPowerMode;
        this.shotgunPowerLatch = shotgunPowerLatch;
    }
}

