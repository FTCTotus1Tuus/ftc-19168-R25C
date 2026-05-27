package org.firstinspires.ftc.teamcode.team.core;

/**
 * Output bundle returned after one TeleOp loop iteration.
 */
public class TeleOpIterationResult {
    public final TeleOpLoopContext context;
    public final TeleOpStatusCoordinator.TraceState traceState;

    public TeleOpIterationResult(TeleOpLoopContext context, TeleOpStatusCoordinator.TraceState traceState) {
        this.context = context;
        this.traceState = traceState;
    }
}

