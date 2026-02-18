package org.firstinspires.ftc.teamcode.team.fsm;

public class TurretFSM {
    private final DarienOpModeFSM opMode;

    /**
     * Constructor for TurretFSM
     * @param opMode The main op mode instance
     */
    public TurretFSM(DarienOpModeFSM opMode) {
        this.opMode = opMode;
    }

    /**
     * Get the current turret heading in degrees based on the servo position.
     * Zero heading is straight ahead (forward motion of the robot)
     * Positive degrees indicate left, negative degrees indicate right.
     * @return The turret heading in degrees.
     */
    public double getTurretHeading(){
        return -900 + DarienOpModeFSM.FIVE_ROTATION_SERVO_SPAN_DEG * opMode.currentTurretPosition;
    }

}


