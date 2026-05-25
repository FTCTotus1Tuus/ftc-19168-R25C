package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.TurretFSM;

/**
 * Coordinates turret manual and odometry aiming behavior.
 */
public class TurretCoordinator {

    private final TurretFSM turretFSM;

    public TurretCoordinator(TurretFSM turretFSM) {
        this.turretFSM = turretFSM;
    }

    public void applyManualOrOdometryControl(
            String autoAlliance,
            double leftStickX,
            double leftTrigger,
            boolean leftStickButton,
            double robotX,
            double robotY,
            double robotHeadingRadians
    ) {
        // Stick direction is checked first; trigger modulates speed within that direction.
        if (leftStickX <= -0.05) {
            if (leftTrigger > 0.25) {
                turretFSM.rotateLeftFast();
            } else {
                turretFSM.rotateLeft();
            }
            turretFSM.setState(TurretFSM.TurretStates.MANUAL);
        } else if (leftStickX >= 0.05) {
            if (leftTrigger > 0.25) {
                turretFSM.rotateRightFast();
            } else {
                turretFSM.rotateRight();
            }
            turretFSM.setState(TurretFSM.TurretStates.MANUAL);
        } else if (leftStickButton) {
            turretFSM.center();
            turretFSM.setState(TurretFSM.TurretStates.MANUAL);
        }
        // Odometry-based turret aiming (when not in manual control)
        else if (turretFSM.getState() == TurretFSM.TurretStates.ODOMETRY) {
            double targetGoalX;
            double targetGoalY;
            if ("RED".equals(autoAlliance)) {
                targetGoalX = DarienOpModeFSM.GOAL_RED_X;
                targetGoalY = DarienOpModeFSM.GOAL_RED_Y;
            } else {
                targetGoalX = DarienOpModeFSM.GOAL_BLUE_X;
                targetGoalY = DarienOpModeFSM.GOAL_BLUE_Y;
            }

            turretFSM.setPositionFromOdometry(targetGoalX, targetGoalY, robotX, robotY, robotHeadingRadians);
        }
    }
}

