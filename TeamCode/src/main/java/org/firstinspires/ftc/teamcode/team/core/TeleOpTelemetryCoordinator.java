package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team.fsm.GateFSM;
import org.firstinspires.ftc.teamcode.team.fsm.IntakeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootingFSM;
import org.firstinspires.ftc.teamcode.team.fsm.TurretFSM;

import java.util.Locale;

/**
 * Coordinates TeleOp telemetry composition.
 */
public class TeleOpTelemetryCoordinator {

    public void addLoopTelemetry(
            Telemetry telemetry,
            GateFSM gateFSM,
            IntakeFSM intakeFSM,
            ShootingFSM shootingFSM,
            TurretFSM turretFSM,
            String shootingPowerMode,
            String shotgunPowerLatch,
            double actualShotgunRpm,
            double ejectionMotorPower,
            double actualShotgunTps,
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
            double runtime
    ) {
        telemetry.addData("GATE: State", gateFSM.getState().toString());
        telemetry.addData("INTAKE: State", intakeFSM.getState().toString());
        telemetry.addData("SHOOTING: Stage", shootingFSM.getStage().toString());

        telemetry.addData("Actual ShotGun RPM", actualShotgunRpm);
        telemetry.addData("ejectionMotor power", ejectionMotorPower);
        telemetry.addData("Actual ShotGun TPS", actualShotgunTps);
        telemetry.addData("Shooting Power Mode", shootingPowerMode);
        telemetry.addData("Shotgun Power Latch", shotgunPowerLatch);

        telemetry.addData("Alliance Color from Auto", autoAlliance);
        telemetry.addData("Target AprilTag ID", targetGoalTagId);
        telemetry.addData("Odometry Pos (X,Y)", String.format(Locale.US, "%.1f, %.1f", robotX, robotY));
        telemetry.addData("Odometry Bearing (deg)", String.format(Locale.US, "%.1f", Math.toDegrees(robotHeadingRadians)));
        telemetry.addData("TURRET: State", turretFSM.getState().toString());
        telemetry.addData("TURRET: Current Turret Pos", turretFSM.getPosition());

        if (isAutoParking) {
            double parkX = "RED".equals(autoAlliance) ? parkRedX : parkBlueX;
            double parkY = "RED".equals(autoAlliance) ? parkRedY : parkBlueY;
            telemetry.addLine(">>> AUTO-PARKING <<<");
            telemetry.addData("Park Target", String.format(Locale.US, "(%.1f, %.1f)", parkX, parkY));
            telemetry.addData("Park Time Remaining", String.format(Locale.US, "%.1fs", autoParkTimeout - (runtime - autoParkStartTime)));
            telemetry.addLine("Move any stick to cancel");
        }
    }
}


