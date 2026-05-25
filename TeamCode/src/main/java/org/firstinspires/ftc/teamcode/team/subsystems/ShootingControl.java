package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team.fsm.ShootingFSM;

/**
 * Shooting sequence contract used by TeleOp coordinators.
 */
public interface ShootingControl {
    boolean isIdle();
    void update(double currentTime, Telemetry telemetry);
    boolean isDone();
    void reset();
    void start(double currentTime, ShootingFSM.PowerLevel powerLevel);
    void finish();
}

