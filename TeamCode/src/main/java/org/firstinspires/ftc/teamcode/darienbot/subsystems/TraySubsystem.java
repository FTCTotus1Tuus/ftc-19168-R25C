package org.firstinspires.ftc.teamcode.darienbot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TraySubsystem {

    public enum Owner {NONE, INTAKE, SCORING}

    // OWNERSHIP STATE
    private Owner owner = Owner.NONE;

    // Software-tracked positions (since servo has no encoder)
    private double currentPosition = 0.5;     // last commanded position
    private double requestedPosition = 0.5;   // last requested target
    private boolean hasRequest = false;

    // HARDWARE COMPONENTS
    private final Servo tray;

    // HARDWARE TUNING CONSTANTS
    public static double TRAY_POS_1_INTAKE = 0.23;
    public static double TRAY_POS_2_INTAKE = 0.8;
    public static double TRAY_POS_3_INTAKE = 0.54;
    public static double TRAY_POS_1_SCORE = .67;
    public static double TRAY_POS_2_SCORE = 0.38;
    public static double TRAY_POS_3_SCORE = 0.08;

    private static final double SERVO_SPEED = 0.02;

    public TraySubsystem(HardwareMap hw) {
        tray = hw.get(Servo.class, "Tray");
        tray.setPosition(currentPosition);
    }

    // -------- Ownership control --------

    /**
     * Request control of the tray. Returns true if granted.
     */
    public boolean requestOwnership(Owner requester) {
        if (owner == Owner.NONE || owner == requester) {
            owner = requester;
            return true;
        }
        return false;
    }

    /**
     * Release ownership
     */
    public void releaseOwnership(Owner requester) {
        if (owner == requester) {
            owner = Owner.NONE;
        }
    }

    // -------- Position control --------

    /**
     * Set tray servo position AND remember it in memory.
     * This is the ONLY place that should call tray.setPosition().
     */
    public void setTrayPosition(double targetPosition) {
        currentPosition = targetPosition;
        tray.setPosition(targetPosition);
    }

    /**
     * Returns the last COMMANDED position, not a hardware readback.
     */
    public double getPosition() {
        return currentPosition;
    }

    /**
     * Called every loop
     */
    public void update() {
        if (!hasRequest) return;

        // Smooth movement so scoring/intake don't jerk the tray
        double diff = requestedPosition - currentPosition;

        if (Math.abs(diff) < SERVO_SPEED) {
            // For simplicity, we treat the move as instantaneous in software:
            currentPosition = requestedPosition;
            tray.setPosition(currentPosition);
            hasRequest = false;  // request completed
        } else {
            // Move a small step toward target
            currentPosition += Math.signum(diff) * SERVO_SPEED;
            tray.setPosition(currentPosition);
        }
    }


    /*
    public void setTrayPosition(String subsystem, int position) {
        double servoPosition = 0;
        switch (subsystem) {
            case "intake":

                switch (position) {
                    case 1:
                        servoPosition = TRAY_POS_1_INTAKE;
                        break;
                    case 2:
                        servoPosition = TRAY_POS_2_INTAKE;
                        break;
                    case 3:
                        servoPosition = TRAY_POS_3_INTAKE;
                        break;
                }

                break;

            case "scoring":

                switch (position) {
                    case 1:
                        servoPosition = TRAY_POS_1_SCORE;
                        break;
                    case 2:
                        servoPosition = TRAY_POS_2_SCORE;
                        break;
                    case 3:
                        servoPosition = TRAY_POS_3_SCORE;
                        break;
                }

                break;
            default:
        }
        tray.setPosition(servoPosition);
    }

     */

}
