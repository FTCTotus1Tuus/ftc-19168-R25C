package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * Coordinates TeleOp loop status telemetry and trace state derivation.
 */
public class TeleOpStatusCoordinator {

    private static final String TRACE_STATE_AUTO_PARK = "AUTO_PARK";
    private static final String TRACE_STATE_DRIVER_CONTROL = "DRIVER_CONTROL";

    public static class TraceState {
        public final String state;
        public final double stateTimerSec;

        public TraceState(String state, double stateTimerSec) {
            this.state = state;
            this.stateTimerSec = stateTimerSec;
        }
    }

    private void addVisionTelemetry(Telemetry telemetry, TurretVisionCoordinator turretVisionCoordinator) {
        telemetry.addData("Vision Status", turretVisionCoordinator.getVisionStatusLine());
        telemetry.addData("Vision Fallback", turretVisionCoordinator.getFallbackStatusLine());
        telemetry.addData("Vision Age (ms)", String.format(Locale.US, "%.0f", turretVisionCoordinator.getLastCameraAgeMs()));
    }

    private TraceState deriveTraceState(TeleOpStatusSnapshot status) {
        String traceState = status.isAutoParking ? TRACE_STATE_AUTO_PARK : TRACE_STATE_DRIVER_CONTROL;
        double traceStateTimer = status.isAutoParking ? (status.currentTime - status.autoParkStartTime) : 0.0;
        return new TraceState(traceState, traceStateTimer);
    }

    public TraceState publishStatus(
            Telemetry telemetry,
            TeleOpTelemetryCoordinator telemetryCoordinator,
            TurretVisionCoordinator turretVisionCoordinator,
            TeleOpStatusSnapshot status
    ) {
        telemetryCoordinator.addLoopTelemetry(telemetry, status);
        addVisionTelemetry(telemetry, turretVisionCoordinator);
        return deriveTraceState(status);
    }
}

