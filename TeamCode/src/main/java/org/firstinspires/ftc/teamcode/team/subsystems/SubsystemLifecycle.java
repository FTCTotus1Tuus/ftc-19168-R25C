package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Common subsystem lifecycle contract for deterministic control-loop orchestration.
 */
public interface SubsystemLifecycle {

    default void init() {
        // Optional stage for subsystems with explicit startup setup.
    }

    default void readSensors(double currentTime, Telemetry telemetry) {
        // Optional stage for subsystems with sensor polling.
    }

    default void update(double currentTime, Telemetry telemetry) {
        // Optional stage for state progression.
    }

    default void writeOutputs(Telemetry telemetry) {
        // Optional stage for actuator output.
    }
}


