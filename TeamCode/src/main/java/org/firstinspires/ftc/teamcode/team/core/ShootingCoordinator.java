package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team.fsm.ShootingFSM;
import org.firstinspires.ftc.teamcode.team.subsystems.GateControl;
import org.firstinspires.ftc.teamcode.team.subsystems.IntakeControl;
import org.firstinspires.ftc.teamcode.team.subsystems.ShootingControl;

/**
 * Coordinates shooting-related interactions across subsystems.
 */
public class ShootingCoordinator {

    private final ShootingControl shootingControl;
    private final IntakeControl intakeControl;
    private final GateControl gateControl;

    public ShootingCoordinator(ShootingControl shootingControl, IntakeControl intakeControl, GateControl gateControl) {
        this.shootingControl = shootingControl;
        this.intakeControl = intakeControl;
        this.gateControl = gateControl;
    }

    public void updateActiveSequence(double currentTime, Telemetry telemetry) {
        if (!shootingControl.isIdle()) {
            shootingControl.update(currentTime, telemetry);
            if (shootingControl.isDone()) {
                shootingControl.reset();
                intakeControl.startIntaking();
            }
        }
    }

    public void handleDriverControls(
            double currentTime,
            boolean closeGateRequested,
            boolean shootPressed,
            boolean shootReleased,
            double rightStickY,
            double shootPowerSelectStickThreshold
    ) {
        if (closeGateRequested) {
            gateControl.close();
        } else if (shootPressed) {
            ShootingFSM.PowerLevel power = (rightStickY < -shootPowerSelectStickThreshold)
                    ? ShootingFSM.PowerLevel.FAR
                    : ShootingFSM.PowerLevel.CLOSE;
            shootingControl.start(currentTime, power);
        } else if (shootReleased) {
            shootingControl.finish();
        }
    }
}

