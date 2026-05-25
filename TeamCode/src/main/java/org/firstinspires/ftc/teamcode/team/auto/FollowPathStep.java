package org.firstinspires.ftc.teamcode.team.auto;

import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;

/**
 * Step to follow a single PathChain with timeout protection.
 * Completes when path is finished or timeout is exceeded.
 */
public class FollowPathStep implements AutoStep {

    private final PathChain pathChain;
    private final double powerMultiplier;
    private final double timeoutSeconds;
    private final boolean isEndPath;  // true if this is the last segment of a multi-part path
    private double stepStartTime;
    private boolean isDone;

    public FollowPathStep(PathChain pathChain, double powerMultiplier, double timeoutSeconds, boolean isEndPath) {
        this.pathChain = pathChain;
        this.powerMultiplier = powerMultiplier;
        this.timeoutSeconds = timeoutSeconds;
        this.isEndPath = isEndPath;
        this.isDone = false;
    }

    public FollowPathStep(PathChain pathChain, double powerMultiplier, double timeoutSeconds) {
        this(pathChain, powerMultiplier, timeoutSeconds, false);
    }

    @Override
    public void init(DarienOpModeFSM opMode) {
        stepStartTime = opMode.getRuntime();
        opMode.follower.setMaxPower(powerMultiplier);
        opMode.follower.followPath(pathChain, isEndPath);
        isDone = false;
    }

    @Override
    public void update(DarienOpModeFSM opMode, double elapsedSec) {
        double stepElapsed = opMode.getRuntime() - stepStartTime;
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
        // Nothing special needed; follower will stop on its own
    }

    @Override
    public String getName() {
        return "FollowPath";
    }
}

