package org.firstinspires.ftc.teamcode.team.auto;

import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootingFSM;

/**
 * Step to shoot and wait for completion.
 * Starts ShootingFSM and completes when it finishes or timeout is exceeded.
 */
public class ShootSequenceStep implements AutoStep {

    private final ShootingFSM.PowerLevel powerLevel;
    private final double timeoutSeconds;
    private double stepStartTime;
    private boolean isDone;

    public ShootSequenceStep(ShootingFSM.PowerLevel powerLevel, double timeoutSeconds) {
        this.powerLevel = powerLevel;
        this.timeoutSeconds = timeoutSeconds;
        this.isDone = false;
    }

    @Override
    public void init(DarienOpModeFSM opMode) {
        stepStartTime = opMode.getRuntime();
        opMode.shootingFSM.start(stepStartTime, powerLevel);
        isDone = false;
    }

    @Override
    public void update(DarienOpModeFSM opMode, double elapsedSec) {
        double stepElapsed = opMode.getRuntime() - stepStartTime;

        // Update shooting FSM on every loop
        opMode.shootingFSM.update(opMode.getRuntime(), opMode.telemetry);

        // Complete when shooting FSM is done or timeout exceeded
        if (!opMode.shootingFSM.isBusy() || stepElapsed > timeoutSeconds) {
            isDone = true;
        }
    }

    @Override
    public boolean isDone() {
        return isDone;
    }

    @Override
    public void cleanup(DarienOpModeFSM opMode) {
        opMode.shootingFSM.reset();
    }

    @Override
    public String getName() {
        return "ShootSequence-" + powerLevel.name();
    }
}

