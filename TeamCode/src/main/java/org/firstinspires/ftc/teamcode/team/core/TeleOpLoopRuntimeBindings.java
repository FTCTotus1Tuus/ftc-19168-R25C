package org.firstinspires.ftc.teamcode.team.core;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team.fsm.GateFSM;
import org.firstinspires.ftc.teamcode.team.fsm.IntakeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootingFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShotgunFSM;
import org.firstinspires.ftc.teamcode.team.fsm.TurretFSM;
import org.firstinspires.ftc.teamcode.team.services.LocalizationService;

/**
 * Bundles runtime service and hardware handles needed by TeleOp loop factories.
 */
public class TeleOpLoopRuntimeBindings {
    public final LocalizationService localizationService;
    public final Follower follower;
    public final IntakeFSM intakeFSM;
    public final ShotgunFSM shotgunFSM;
    public final ShootingFSM shootingFSM;
    public final TurretFSM turretFSM;
    public final GateFSM gateFSM;
    public final Telemetry telemetry;

    public TeleOpLoopRuntimeBindings(
            LocalizationService localizationService,
            Follower follower,
            IntakeFSM intakeFSM,
            ShotgunFSM shotgunFSM,
            ShootingFSM shootingFSM,
            TurretFSM turretFSM,
            GateFSM gateFSM,
            Telemetry telemetry
    ) {
        this.localizationService = localizationService;
        this.follower = follower;
        this.intakeFSM = intakeFSM;
        this.shotgunFSM = shotgunFSM;
        this.shootingFSM = shootingFSM;
        this.turretFSM = turretFSM;
        this.gateFSM = gateFSM;
        this.telemetry = telemetry;
    }
}

