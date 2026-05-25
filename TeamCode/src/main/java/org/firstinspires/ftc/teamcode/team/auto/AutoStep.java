package org.firstinspires.ftc.teamcode.team.auto;

import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;

/**
 * Base interface for composable autonomous steps.
 * Each step encapsulates a single high-level action (follow path, shoot, intake, etc.)
 * and tracks completion via lifecycle methods.
 */
public interface AutoStep {

    /**
     * Called once when the step becomes active.
     * Use for initialization (e.g., starting motors, setting modes).
     */
    void init(DarienOpModeFSM opMode);

    /**
     * Called every control loop while the step is active.
     * Use for continuous updates (e.g., checking follower.isBusy(), updating subsystem FSMs).
     */
    void update(DarienOpModeFSM opMode, double elapsedSec);

    /**
     * @return true if the step has completed (wait for next step), false otherwise.
     */
    boolean isDone();

    /**
     * Called once when the step is transitioning out (either completed or timed out).
     * Use for cleanup (e.g., stopping motors, resetting flags).
     */
    void cleanup(DarienOpModeFSM opMode);

    /**
     * @return Human-readable name for telemetry/logging.
     */
    String getName();
}

