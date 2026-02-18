package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.config.Config;

@Config

public class ShootArtifactFSM {

    public enum ShootingStage {
        IDLE,
        SHOTGUN_SPINUP,
        ELEVATOR_UP,
        ELEVATOR_DOWN,
        FINISHED
    }

    private final DarienOpModeFSM opMode;
    private boolean ejectionMotorsControlledByPattern = false;

    private ShootingStage shootingStage = ShootingStage.IDLE;
    private double shootingStartTime = 0;
    private double shootingPower = 0;

    // Timings (seconds)
    public static double ELEVATOR_UP_DELAY = .35;    // elevator up delay
    public static double ELEVATOR_DOWN_DELAY = .25;   //elevator down delay
    public static double SPINUP_DELAY = 0.0;    // shotgun running before elevator up

    public ShootArtifactFSM(DarienOpModeFSM opMode) {
        this.opMode = opMode;
    }

    // Call this to begin shooting
    public void startShooting(double shootingPower) {
        this.shootingPower = shootingPower;
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
                    shootingStage = ShootingStage.ELEVATOR_UP;
                }
                break;

            case ELEVATOR_UP:
                opMode.Elevator.setPosition(DarienOpModeFSM.ELEVATOR_POS_UP);
                if (currentTime - shootingStartTime >= ELEVATOR_UP_DELAY) {
                    shootingStage = ShootingStage.ELEVATOR_DOWN;
                    shootingStartTime = currentTime; // Reset timer for next stage
                }
                break;

            case ELEVATOR_DOWN:
                opMode.Elevator.setPosition(DarienOpModeFSM.ELEVATOR_POS_DOWN);
                if (currentTime - shootingStartTime >= ELEVATOR_DOWN_DELAY) {
                    if (!ejectionMotorsControlledByPattern) {
                        shotGunStop();
                    }
                    shootingStage = ShootingStage.FINISHED;
                }
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
        if (power == opMode.SHOT_GUN_POWER_UP) {
            shotGunRPM(opMode.SHOT_GUN_POWER_UP_RPM_AUTO);
        } else if (power == opMode.SHOT_GUN_POWER_UP_FAR) {
            shotGunRPM(opMode.SHOT_GUN_POWER_UP_FAR_RPM_AUTO);
        }
    }

    public void shotGunTeleop(double power) {
        //opMode.ejectionMotor.setPower(opMode.getVoltageAdjustedMotorPower(power));
        if (power == opMode.SHOT_GUN_POWER_UP) {
            shotGunRPM(opMode.SHOT_GUN_POWER_UP_RPM);
        } else if (power == opMode.SHOT_GUN_POWER_UP_FAR) {
            shotGunRPM(opMode.SHOT_GUN_POWER_UP_FAR_RPM_TELEOP);
        }
    }

    public void shotGunRPM(double RPM) {
        opMode.ejectionMotor.setVelocity(opMode.getTicksPerSecond(RPM));
    }

    public void shotGunStop() {
        opMode.ejectionMotor.setPower(0);
    }

    public void setEjectionMotorsControlledByPattern(boolean controlled) {
        this.ejectionMotorsControlledByPattern = controlled;
    }

    public ShootingStage getShootingStage() {
        return shootingStage;
    }

}
