package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.teamcode.team.fsm.GateFSM;
import org.firstinspires.ftc.teamcode.team.fsm.IntakeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootingFSM;
import org.firstinspires.ftc.teamcode.team.fsm.TurretFSM;

/**
 * Immutable snapshot of TeleOp status data for one loop iteration.
 *
 * This keeps method signatures short and helps new developers reason about
 * "what data belongs to loop status" in one place.
 */
public class TeleOpStatusSnapshot {
    public final GateFSM gateFSM;
    public final IntakeFSM intakeFSM;
    public final ShootingFSM shootingFSM;
    public final TurretFSM turretFSM;

    public final String shootingPowerMode;
    public final String shotgunPowerLatch;

    public final double ejectionMotorRpm;
    public final double ejectionMotorPower;
    public final double ejectionMotorVelocity;

    public final String autoAlliance;
    public final int targetGoalTagId;

    public final double robotX;
    public final double robotY;
    public final double robotHeadingRadians;

    public final boolean isAutoParking;
    public final double parkRedX;
    public final double parkRedY;
    public final double parkBlueX;
    public final double parkBlueY;
    public final double autoParkTimeout;
    public final double autoParkStartTime;
    public final double currentTime;

    public TeleOpStatusSnapshot(
            GateFSM gateFSM,
            IntakeFSM intakeFSM,
            ShootingFSM shootingFSM,
            TurretFSM turretFSM,
            String shootingPowerMode,
            String shotgunPowerLatch,
            double ejectionMotorRpm,
            double ejectionMotorPower,
            double ejectionMotorVelocity,
            String autoAlliance,
            int targetGoalTagId,
            double robotX,
            double robotY,
            double robotHeadingRadians,
            boolean isAutoParking,
            double parkRedX,
            double parkRedY,
            double parkBlueX,
            double parkBlueY,
            double autoParkTimeout,
            double autoParkStartTime,
            double currentTime
    ) {
        this.gateFSM = gateFSM;
        this.intakeFSM = intakeFSM;
        this.shootingFSM = shootingFSM;
        this.turretFSM = turretFSM;
        this.shootingPowerMode = shootingPowerMode;
        this.shotgunPowerLatch = shotgunPowerLatch;
        this.ejectionMotorRpm = ejectionMotorRpm;
        this.ejectionMotorPower = ejectionMotorPower;
        this.ejectionMotorVelocity = ejectionMotorVelocity;
        this.autoAlliance = autoAlliance;
        this.targetGoalTagId = targetGoalTagId;
        this.robotX = robotX;
        this.robotY = robotY;
        this.robotHeadingRadians = robotHeadingRadians;
        this.isAutoParking = isAutoParking;
        this.parkRedX = parkRedX;
        this.parkRedY = parkRedY;
        this.parkBlueX = parkBlueX;
        this.parkBlueY = parkBlueY;
        this.autoParkTimeout = autoParkTimeout;
        this.autoParkStartTime = autoParkStartTime;
        this.currentTime = currentTime;
    }
}

