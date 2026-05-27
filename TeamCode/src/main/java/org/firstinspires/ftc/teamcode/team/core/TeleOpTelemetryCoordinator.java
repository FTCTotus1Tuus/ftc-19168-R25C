package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * Coordinates TeleOp telemetry composition.
 */
public class TeleOpTelemetryCoordinator {

    private void addSubsystemStateTelemetry(Telemetry telemetry, TeleOpStatusSnapshot status) {
        telemetry.addData("GATE: State", status.gateFSM.getState().toString());
        telemetry.addData("INTAKE: State", status.intakeFSM.getState().toString());
        telemetry.addData("SHOOTING: Stage", status.shootingFSM.getStage().toString());
    }

    private void addShooterTelemetry(Telemetry telemetry, TeleOpStatusSnapshot status) {
        telemetry.addData("Actual ShotGun RPM", status.ejectionMotorRpm);
        telemetry.addData("ejectionMotor power", status.ejectionMotorPower);
        telemetry.addData("Actual ShotGun TPS", status.ejectionMotorVelocity);
        telemetry.addData("Shooting Power Mode", status.shootingPowerMode);
        telemetry.addData("Shotgun Power Latch", status.shotgunPowerLatch);
    }

    private void addPoseAndTargetTelemetry(Telemetry telemetry, TeleOpStatusSnapshot status) {
        telemetry.addData("Alliance Color from Auto", status.autoAlliance);
        telemetry.addData("Target AprilTag ID", status.targetGoalTagId);
        telemetry.addData("Odometry Pos (X,Y)", String.format(Locale.US, "%.1f, %.1f", status.robotX, status.robotY));
        telemetry.addData("Odometry Bearing (deg)", String.format(Locale.US, "%.1f", Math.toDegrees(status.robotHeadingRadians)));
        telemetry.addData("TURRET: State", status.turretFSM.getState().toString());
        telemetry.addData("TURRET: Current Turret Pos", status.turretFSM.getPosition());
    }

    private void addAutoParkTelemetry(Telemetry telemetry, TeleOpStatusSnapshot status) {
        double parkX = "RED".equals(status.autoAlliance) ? status.parkRedX : status.parkBlueX;
        double parkY = "RED".equals(status.autoAlliance) ? status.parkRedY : status.parkBlueY;
        telemetry.addLine(">>> AUTO-PARKING <<<");
        telemetry.addData("Park Target", String.format(Locale.US, "(%.1f, %.1f)", parkX, parkY));
        telemetry.addData("Park Time Remaining", String.format(Locale.US, "%.1fs", status.autoParkTimeout - (status.currentTime - status.autoParkStartTime)));
        telemetry.addLine("Move any stick to cancel");
    }

    public void addLoopTelemetry(
            Telemetry telemetry,
            TeleOpStatusSnapshot status
    ) {
        addSubsystemStateTelemetry(telemetry, status);
        addShooterTelemetry(telemetry, status);
        addPoseAndTargetTelemetry(telemetry, status);

        if (status.isAutoParking) {
            addAutoParkTelemetry(telemetry, status);
        }
    }
}


