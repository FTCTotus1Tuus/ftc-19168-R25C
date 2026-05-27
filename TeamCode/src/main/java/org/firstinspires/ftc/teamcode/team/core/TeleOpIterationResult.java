package org.firstinspires.ftc.teamcode.team.core;

/**
 * Output bundle returned after one TeleOp loop iteration.
 */
public class TeleOpIterationResult {
    public final TeleOpLoopState state;
    public final TeleOpStatusCoordinator.TraceState traceState;

    public TeleOpIterationResult(TeleOpLoopState state, TeleOpStatusCoordinator.TraceState traceState) {
        this.state = state;
        this.traceState = traceState;
    }
}

