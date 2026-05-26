package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team.fsm.GateFSM;
import org.firstinspires.ftc.teamcode.team.fsm.IntakeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootingFSM;
import org.firstinspires.ftc.teamcode.team.fsm.TurretFSM;

import java.util.Locale;

/**
 * Coordinates TeleOp loop status telemetry and trace state derivation.
 */
public class TeleOpStatusCoordinator {

    public static class TraceState {
        public final String state;
        public final double stateTimerSec;

        public TraceState(String state, double stateTimerSec) {
            this.state = state;
            this.stateTimerSec = stateTimerSec;
        }
    }

    public TraceState publishStatus(
            Telemetry telemetry,
            TeleOpTelemetryCoordinator telemetryCoordinator,
            TurretVisionCoordinator turretVisionCoordinator,
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
            double robotX,
            double robotY,
            double robotHeadingRadians,
            boolean isAutoParking,
            int targetGoalTagId,
            double parkRedX,
            double parkRedY,
            double parkBlueX,
            double parkBlueY,
            double autoParkTimeout,
            double autoParkStartTime,
            double currentTime
    ) {
        telemetryCoordinator.addLoopTelemetry(
                telemetry,
                gateFSM,
                intakeFSM,
                shootingFSM,
                turretFSM,
                shootingPowerMode,
                shotgunPowerLatch,
                ejectionMotorRpm,
                ejectionMotorPower,
                ejectionMotorVelocity,
                autoAlliance,
                targetGoalTagId,
                robotX,
                robotY,
                robotHeadingRadians,
                isAutoParking,
                parkRedX,
                parkRedY,
                parkBlueX,
                parkBlueY,
                autoParkTimeout,
                autoParkStartTime,
                currentTime
        );

        telemetry.addData("Vision Status", turretVisionCoordinator.getVisionStatusLine());
        telemetry.addData("Vision Fallback", turretVisionCoordinator.getFallbackStatusLine());
        telemetry.addData("Vision Age (ms)", String.format(Locale.US, "%.0f", turretVisionCoordinator.getLastCameraAgeMs()));

        String traceState = isAutoParking ? "AUTO_PARK" : "DRIVER_CONTROL";
        double traceStateTimer = isAutoParking ? (currentTime - autoParkStartTime) : 0.0;
        return new TraceState(traceState, traceStateTimer);
    }
}


