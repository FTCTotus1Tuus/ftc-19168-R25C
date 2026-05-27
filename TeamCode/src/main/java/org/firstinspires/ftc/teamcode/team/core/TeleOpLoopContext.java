package org.firstinspires.ftc.teamcode.team.core;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team.fsm.GateFSM;
import org.firstinspires.ftc.teamcode.team.fsm.IntakeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootingFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShotgunFSM;
import org.firstinspires.ftc.teamcode.team.fsm.TurretFSM;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;
import org.firstinspires.ftc.teamcode.team.services.LocalizationService;

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
            LocalizationService localizationService,
            Follower follower,
            IntakeFSM intakeFSM,
            ShotgunFSM shotgunFSM,
            ShootingFSM shootingFSM,
            TurretFSM turretFSM,
            GateFSM gateFSM,
            Telemetry telemetry,
            String autoAlliance,
            DarienOpModeFSM.ShootingPowerModes shootingPowerMode,
            TeleOpLoopConfig config
    ) {
        return new TeleOpLoopContext(
                TeleOpLoopState.initial(autoAlliance, shootingPowerMode),
                TeleOpLoopDependencies.create(
                        coordinators,
                        localizationService,
                        follower,
                        intakeFSM,
                        shotgunFSM,
                        shootingFSM,
                        turretFSM,
                        gateFSM,
                        telemetry,
                        config
                )
        );
    }

    public TeleOpLoopContext withState(TeleOpLoopState newState) {
        return new TeleOpLoopContext(newState, dependencies);
    }
}



