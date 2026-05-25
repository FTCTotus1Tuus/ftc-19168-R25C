package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.TurretFSM;

/**
 * Coordinates turret mode transitions requested by driver controls.
 */
public class TurretModeCoordinator {

    public static class ModeSwitchResult {
        public final DarienOpModeFSM.ShootingPowerModes shootingPowerMode;
        public final boolean shouldStartGoalReading;

        public ModeSwitchResult(DarienOpModeFSM.ShootingPowerModes shootingPowerMode, boolean shouldStartGoalReading) {
            this.shootingPowerMode = shootingPowerMode;
            this.shouldStartGoalReading = shouldStartGoalReading;
        }
    }

    private final TurretFSM turretFSM;

    public TurretModeCoordinator(TurretFSM turretFSM) {
        this.turretFSM = turretFSM;
    }

    public ModeSwitchResult handleModeSwitches(
            boolean dpadUpPressed,
            boolean dpadDownPressed,
            DarienOpModeFSM.ShootingPowerModes currentPowerMode
    ) {
        DarienOpModeFSM.ShootingPowerModes nextPowerMode = currentPowerMode;
        boolean shouldStartGoalReading = false;

        if (dpadUpPressed) {
            turretFSM.setState(TurretFSM.TurretStates.ODOMETRY);
            nextPowerMode = DarienOpModeFSM.ShootingPowerModes.ODOMETRY;
        } else if (dpadDownPressed) {
            turretFSM.setState(TurretFSM.TurretStates.CAMERA);
            shouldStartGoalReading = true;
        }

        return new ModeSwitchResult(nextPowerMode, shouldStartGoalReading);
    }
}

