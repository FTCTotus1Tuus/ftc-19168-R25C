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
            double autoParkStickDeadzone,
            double autoParkTimeout,
            double driveDeadzone,
            double inputExponent,
            double speedScale,
            double speedScaleTurn,
            double rotationScale,
            double parkRedX,
            double parkRedY,
            double parkRedHeadingDeg,
            double parkBlueX,
            double parkBlueY,
            double parkBlueHeadingDeg,
            double autoParkPower,
            double humanPlayerRedX,
            double humanPlayerRedY,
            double humanPlayerBlueX,
            double humanPlayerBlueY,
            double robotCenterOffsetX,
            double robotCenterOffsetY,
            double shootingPowerOdometryYThreshold,
            double shootPowerSelectStickThreshold,
            double closeRpm,
            double farRpm
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
                        autoParkStickDeadzone,
                        autoParkTimeout,
                        driveDeadzone,
                        inputExponent,
                        speedScale,
                        speedScaleTurn,
                        rotationScale,
                        parkRedX,
                        parkRedY,
                        parkRedHeadingDeg,
                        parkBlueX,
                        parkBlueY,
                        parkBlueHeadingDeg,
                        autoParkPower,
                        humanPlayerRedX,
                        humanPlayerRedY,
                        humanPlayerBlueX,
                        humanPlayerBlueY,
                        robotCenterOffsetX,
                        robotCenterOffsetY,
                        shootingPowerOdometryYThreshold,
                        shootPowerSelectStickThreshold,
                        closeRpm,
                        farRpm
                )
        );
    }

    public TeleOpLoopContext withState(TeleOpLoopState newState) {
        return new TeleOpLoopContext(newState, dependencies);
    }
}



