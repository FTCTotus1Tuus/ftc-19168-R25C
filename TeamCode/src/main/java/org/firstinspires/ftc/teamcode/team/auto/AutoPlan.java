package org.firstinspires.ftc.teamcode.team.auto;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;

/**
 * Orchestrator for a sequence of AutoSteps.
 * Manages step lifecycle, transitions, and telemetry.
 *
 * Usage:
 *   AutoPlan plan = new AutoPlan()
 *     .add(new FollowPathStep(...))
 *     .add(new ShootSequenceStep(...))
 *     .add(new CleanupStep());
 *
 *   // In loop:
 *   plan.update(opMode, elapsedTime);
 */
public class AutoPlan {

    private final List<AutoStep> steps;
    private int currentStepIndex;
    private AutoStep currentStep;
    private boolean isComplete;
    private long stepStartTimeNanos;

    public AutoPlan() {
        this.steps = new ArrayList<>();
        this.currentStepIndex = 0;
        this.currentStep = null;
        this.isComplete = false;
        this.stepStartTimeNanos = 0;
    }

    /**
     * Add a step to the plan (fluent interface).
     */
    public AutoPlan add(AutoStep step) {
        steps.add(step);
        return this;
    }

    /**
     * Initialize the plan (called once at start).
     */
    public void init(DarienOpModeFSM opMode) {
        if (!steps.isEmpty()) {
            currentStepIndex = 0;
            currentStep = steps.get(0);
            stepStartTimeNanos = System.nanoTime();
            currentStep.init(opMode);
        } else {
            isComplete = true;
        }
    }

    /**
     * Update for one control loop iteration.
     * Automatically handles step transitions when isDone() is true.
     */
    public void update(DarienOpModeFSM opMode) {
        if (isComplete || currentStep == null) {
            return;
        }

        double elapsedSec = (System.nanoTime() - stepStartTimeNanos) / 1e9;
        currentStep.update(opMode, elapsedSec);

        // Check if current step is complete
        if (currentStep.isDone()) {
            currentStep.cleanup(opMode);

            // Move to next step
            currentStepIndex++;
            if (currentStepIndex < steps.size()) {
                currentStep = steps.get(currentStepIndex);
                stepStartTimeNanos = System.nanoTime();
                currentStep.init(opMode);
            } else {
                isComplete = true;
                currentStep = null;
            }
        }
    }

    /**
     * @return true if all steps have completed.
     */
    public boolean isComplete() {
        return isComplete;
    }

    /**
     * @return Human-readable info for telemetry (step number, name, etc.).
     */
    public String getStatus() {
        if (currentStep == null) {
            return "Plan Complete";
        }
        return String.format("Step %d/%d: %s",
                currentStepIndex + 1,
                steps.size(),
                currentStep.getName());
    }

    /**
     * Stop the plan early and clean up (optional, for error recovery).
     */
    public void stop(DarienOpModeFSM opMode) {
        if (currentStep != null) {
            currentStep.cleanup(opMode);
        }
        isComplete = true;
        currentStep = null;
    }
}

