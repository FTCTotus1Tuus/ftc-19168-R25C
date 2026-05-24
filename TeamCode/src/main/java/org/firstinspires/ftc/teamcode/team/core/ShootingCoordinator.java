package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team.fsm.IntakeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootingFSM;
import org.firstinspires.ftc.teamcode.team.fsm.TeleOpFSM;
import org.firstinspires.ftc.teamcode.team.subsystems.GateControl;

/**
 * Coordinates shooting-related interactions across subsystems.
 */
public class ShootingCoordinator {

    private final ShootingFSM shootingFSM;
    private final IntakeFSM intakeFSM;
    private final GateControl gateControl;

    public ShootingCoordinator(ShootingFSM shootingFSM, IntakeFSM intakeFSM, GateControl gateControl) {
        this.shootingFSM = shootingFSM;
        this.intakeFSM = intakeFSM;
        this.gateControl = gateControl;
    }

    public void updateActiveSequence(double currentTime, Telemetry telemetry) {
        if (shootingFSM.getStage() != ShootingFSM.Stage.IDLE) {
            shootingFSM.update(currentTime, telemetry);
            if (shootingFSM.isDone()) {
                shootingFSM.reset();
                intakeFSM.startIntaking();
            }
        }
    }

    public void handleDriverControls(
            double currentTime,
            boolean closeGateRequested,
            boolean shootPressed,
            boolean shootReleased,
            double rightStickY
    ) {
        if (closeGateRequested) {
            gateControl.close();
        } else if (shootPressed) {
            ShootingFSM.PowerLevel power = (rightStickY < -TeleOpFSM.SHOOT_POWER_SELECT_STICK_THRESHOLD)
                    ? ShootingFSM.PowerLevel.FAR
                    : ShootingFSM.PowerLevel.CLOSE;
            shootingFSM.start(currentTime, power);
        } else if (shootReleased) {
            shootingFSM.finish();
        }
    }
}

