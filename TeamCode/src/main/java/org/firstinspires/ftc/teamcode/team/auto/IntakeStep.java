package org.firstinspires.ftc.teamcode.team.auto;

import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;

/**
 * Step that combines intake with path following.
 * Starts intake, follows a path at reduced speed, and completes when path finishes or timeout occurs.
 */
public class IntakeStep implements AutoStep {

    private final PathChain pathChain;
    private final double powerMultiplier;
    private final double timeoutSeconds;
    private final boolean isEndPath;
    private double stepStartTime;
    private boolean isDone;

    public IntakeStep(PathChain pathChain, double powerMultiplier, double timeoutSeconds, boolean isEndPath) {
        this.pathChain = pathChain;
        this.powerMultiplier = powerMultiplier;
        this.timeoutSeconds = timeoutSeconds;
        this.isEndPath = isEndPath;
        this.isDone = false;
    }

    public IntakeStep(PathChain pathChain, double powerMultiplier, double timeoutSeconds) {
        this(pathChain, powerMultiplier, timeoutSeconds, false);
    }

    @Override
    public void init(DarienOpModeFSM opMode) {
        stepStartTime = opMode.getRuntime();
        opMode.intakeFSM.startIntaking();
        opMode.follower.setMaxPower(powerMultiplier);
        opMode.follower.followPath(pathChain, isEndPath);
        isDone = false;
    }

    @Override
    public void update(DarienOpModeFSM opMode, double elapsedSec) {
        double stepElapsed = opMode.getRuntime() - stepStartTime;

        // Keep updating intake FSM
        opMode.intakeFSM.updateIntaking(opMode.getRuntime(), true, opMode.telemetry);

        // Complete when path is done or timeout exceeded
        if (!opMode.follower.isBusy() || stepElapsed > timeoutSeconds) {
            isDone = true;
        }
    }

    @Override
    public boolean isDone() {
        return isDone;
    }

    @Override
    public void cleanup(DarienOpModeFSM opMode) {
        opMode.intakeFSM.off();
    }

    @Override
    public String getName() {
        return "IntakePath";
    }
}

