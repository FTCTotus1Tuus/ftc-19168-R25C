package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.config.Config;

@Config

public class ShootArtifactFSM {

    public enum ShootingStage {
        IDLE,
        SHOTGUN_SPINUP,
        GATE_OPEN,
        FINISHED
    }

    private final DarienOpModeFSM opMode;
    private boolean ejectionMotorsControlledByPattern = false;

    private ShootingStage shootingStage = ShootingStage.IDLE;
    private double shootingStartTime = 0;

    // Timings (seconds)
    public static double GATE_OPEN_DELAY = .35;
    public static double SPINUP_DELAY = 0.0;    // shotgun running before shooting artifact

    public ShootArtifactFSM(DarienOpModeFSM opMode) {
        this.opMode = opMode;
    }

    // Call this to begin shooting
    public void startShooting(double shootingPower) {
        if (!ejectionMotorsControlledByPattern) {
            shotGun(shootingPower);
        }

        shootingStartTime = opMode.getRuntime();
        shootingStage = ShootingStage.SHOTGUN_SPINUP;
    }

    // Call this inside loop() or inside your main auto while-loop
    public void updateShooting() {
        if (shootingStage == ShootingStage.IDLE ||
                shootingStage == ShootingStage.FINISHED) {
            return;
        }

        double currentTime = opMode.getRuntime();

        switch (shootingStage) {

            case SHOTGUN_SPINUP:
                // If the pattern has already spun up the shotgun, there's no need to wait for the delay here.
                if (ejectionMotorsControlledByPattern || currentTime - shootingStartTime >= SPINUP_DELAY) {
                    shootingStartTime = currentTime; // Reset timer for next stage
                    shootingStage = ShootingStage.GATE_OPEN;
                }
                break;

            case GATE_OPEN:
                //opMode.Elevator.setPosition(DarienOpModeFSM.GATE_OPEN);
                if (currentTime - shootingStartTime >= GATE_OPEN_DELAY) {
                    shootingStage = ShootingStage.FINISHED;
                    shootingStartTime = currentTime; // Reset timer for next stage
                }
                break;
            case IDLE:
            default:
                break;
        }
    }

    // Use this to check if it is done
    public boolean shootingDone() {
        return shootingStage == ShootingStage.FINISHED;
    }

    // If you want a hard reset:
    public void resetShooting() {
        shootingStage = ShootingStage.IDLE;
    }

    public void shotGun(double power) {
        //opMode.ejectionMotor.setPower(opMode.getVoltageAdjustedMotorPower(power));
        if (power == DarienOpModeFSM.SHOT_GUN_POWER_UP) {
            shotGunRPM(DarienOpModeFSM.SHOT_GUN_POWER_UP_RPM_AUTO);
        } else if (power == DarienOpModeFSM.SHOT_GUN_POWER_UP_FAR) {
            shotGunRPM(DarienOpModeFSM.SHOT_GUN_POWER_UP_FAR_RPM_AUTO);
        }
    }

    public void shotGunRPM(double RPM) {
        opMode.ejectionMotor.setVelocity(opMode.getTicksPerSecond(RPM));
    }

    public void setEjectionMotorsControlledByPattern(boolean controlled) {
        this.ejectionMotorsControlledByPattern = controlled;
    }


}
