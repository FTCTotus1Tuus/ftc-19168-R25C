package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShotgunFSM;

/**
 * Coordinates shooter power-mode and latch-selection decisions.
 */
public class ShooterPowerCoordinator {

    public static class PowerState {
        public final DarienOpModeFSM.ShotgunPowerLevel latch;
        public final DarienOpModeFSM.ShootingPowerModes mode;

        public PowerState(DarienOpModeFSM.ShotgunPowerLevel latch, DarienOpModeFSM.ShootingPowerModes mode) {
            this.latch = latch;
            this.mode = mode;
        }
    }

    public PowerState computePowerState(
            DarienOpModeFSM.ShootingPowerModes currentMode,
            DarienOpModeFSM.ShotgunPowerLevel currentLatch,
            double robotY,
            double odometryYThreshold,
            double rightStickY,
            double stickThreshold,
            boolean rightStickButtonPressed,
            boolean buttonAPressed
    ) {
        DarienOpModeFSM.ShotgunPowerLevel nextLatch = currentLatch;
        DarienOpModeFSM.ShootingPowerModes nextMode = currentMode;

        // ODOMETRY mode auto-selects FAR/CLOSE based on robot Y.
        if (currentMode == DarienOpModeFSM.ShootingPowerModes.ODOMETRY) {
            nextLatch = (robotY <= odometryYThreshold)
                    ? DarienOpModeFSM.ShotgunPowerLevel.HIGH
                    : DarienOpModeFSM.ShotgunPowerLevel.LOW;
        }

        // Manual input overrides auto mode and directly selects latch.
        if (rightStickY < -stickThreshold) {
            nextLatch = DarienOpModeFSM.ShotgunPowerLevel.HIGH;
            nextMode = DarienOpModeFSM.ShootingPowerModes.MANUAL;
        } else if (rightStickY > stickThreshold) {
            nextLatch = DarienOpModeFSM.ShotgunPowerLevel.LOW;
            nextMode = DarienOpModeFSM.ShootingPowerModes.MANUAL;
        } else if (rightStickButtonPressed || buttonAPressed) {
            nextLatch = DarienOpModeFSM.ShotgunPowerLevel.OFF;
            nextMode = DarienOpModeFSM.ShootingPowerModes.MANUAL;
        }

        return new PowerState(nextLatch, nextMode);
    }

    public void applyRequestedPower(
            ShotgunFSM shotgunFSM,
            DarienOpModeFSM.ShotgunPowerLevel latch,
            double closeRpm,
            double farRpm,
            Telemetry telemetry
    ) {
        switch (latch) {
            case OFF:
                shotgunFSM.toOff();
                telemetry.addData("Requested ShotGun RPM", 0);
                break;
            case HIGH:
                shotgunFSM.toPowerUpFar(farRpm);
                telemetry.addData("Requested ShotGun RPM", farRpm);
                break;
            case LOW:
            default:
                shotgunFSM.toPowerUp(closeRpm);
                telemetry.addData("Requested ShotGun RPM", closeRpm);
                break;
        }
    }
}

