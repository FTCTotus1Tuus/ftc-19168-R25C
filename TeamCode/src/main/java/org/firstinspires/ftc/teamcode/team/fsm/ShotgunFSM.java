package org.firstinspires.ftc.teamcode.team.fsm;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.team.MotorHelper;

public class ShotgunFSM {

    private final DarienOpModeFSM opmode;
    private final MotorHelper MotorHelper;
    private final double SHOT_GUN_POWER_UP;
    private final double SHOT_GUN_POWER_UP_FAR;
    private final DcMotorEx shotgunMotor;
    double[] pidOutput = {0, 0, 0, 0};

    public enum State {
        OFF,
        POWER_UP,
        POWER_UP_FAR;
    }

    private State current = State.OFF;

    public ShotgunFSM(double powerlow, double powerhigh, DcMotorEx motor, DarienOpModeFSM opmode, MotorHelper motorHelper) {
        this.SHOT_GUN_POWER_UP = powerlow;
        this.SHOT_GUN_POWER_UP_FAR = powerhigh;
        this.shotgunMotor = motor;
        this.opmode = opmode;
        MotorHelper = motorHelper;
    }

    /** Set the desired state explicitly. */
    public void setState(State state) {
        this.current = state;
    }

    /** Get the current state. */
    public State getState() {
        return current;
    }

    /** Convenience methods for transitions. */
    public void toOff() {
        current = State.OFF;
        shotgunMotor.setPower(0);
    }

    public void toPowerUp(double power) {
        current = State.POWER_UP;
        //shotgunMotor.setPower(opmode.getVoltageAdjustedMotorPower(SHOT_GUN_POWER_UP));
        //shotgunMotor.setVelocity(opmode.getTicksPerSecond(opmode.SHOT_GUN_POWER_UP_RPM));
        pidOutput = MotorHelper.pidRunWithEncoder(
                shotgunMotor,
                opmode.SHOT_GUN_PGAIN,
                opmode.SHOT_GUN_PGAIN2,
                opmode.SHOT_GUN_IGAIN,
                power,
                opmode.SHOT_GUN_PDUTY_MIN,
                opmode.SHOT_GUN_PDUTY_MAX,
                opmode.SHOT_GUN_IDUTY_MIN,
                opmode.SHOT_GUN_IDUTY_MAX,
                pidOutput[1],
                opmode.SHOT_GUN_POWER_MIN,
                opmode.SHOT_GUN_POWER_MAX,
                opmode.SHOT_GUN_GAIN,
                opmode.SHOT_GUN_MIN_RPM,
                opmode.SHOT_GUN_MAX_RPM,
                0,
                true
        );
        shotgunMotor.setPower(pidOutput[0]);
    }

    public void toPowerUpFar(double power) {
        current = State.POWER_UP_FAR;
        //shotgunMotor.setPower(opmode.getVoltageAdjustedMotorPower(SHOT_GUN_POWER_UP_FAR));
        //shotgunMotor.setVelocity(opmode.getTicksPerSecond(power));
        pidOutput = MotorHelper.pidRunWithEncoder(
                shotgunMotor,
                opmode.SHOT_GUN_PGAIN,
                opmode.SHOT_GUN_PGAIN2,
                opmode.SHOT_GUN_IGAIN,
                power,
                opmode.SHOT_GUN_PDUTY_MIN,
                opmode.SHOT_GUN_PDUTY_MAX,
                opmode.SHOT_GUN_IDUTY_MIN,
                opmode.SHOT_GUN_IDUTY_MAX,
                pidOutput[1],
                opmode.SHOT_GUN_POWER_MIN,
                opmode.SHOT_GUN_POWER_MAX,
                opmode.SHOT_GUN_GAIN,
                opmode.SHOT_GUN_MIN_RPM,
                opmode.SHOT_GUN_MAX_RPM,
                0,
                true
        );
        shotgunMotor.setPower(pidOutput[0]);
    }

}
