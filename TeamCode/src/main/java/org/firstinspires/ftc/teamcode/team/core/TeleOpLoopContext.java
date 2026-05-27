package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;

/**
 * Bundles the mutable TeleOp loop state and stable dependencies into one object.
 */
public class TeleOpLoopContext {
    public final TeleOpLoopState state;
    public final TeleOpLoopDependencies dependencies;

    public TeleOpLoopContext(TeleOpLoopState state, TeleOpLoopDependencies dependencies) {
        this.state = state;
        this.dependencies = dependencies;
    }

    public static TeleOpLoopContext create(
            TeleOpCoordinatorSet coordinators,
            TeleOpLoopRuntimeBindings runtime,
            String autoAlliance,
            DarienOpModeFSM.ShootingPowerModes shootingPowerMode,
            TeleOpLoopConfig config
    ) {
        return new TeleOpLoopContext(
                TeleOpLoopState.initial(autoAlliance, shootingPowerMode),
                TeleOpLoopDependencies.create(
                        coordinators,
                        runtime,
                        config
                )
        );
    }

    public TeleOpLoopContext withState(TeleOpLoopState newState) {
        return new TeleOpLoopContext(newState, dependencies);
    }
}



