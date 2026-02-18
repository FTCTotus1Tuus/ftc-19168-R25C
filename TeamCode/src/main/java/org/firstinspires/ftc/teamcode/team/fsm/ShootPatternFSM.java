package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.ArrayList;

@Config
public class ShootPatternFSM {

    public enum Stage {
        IDLE,
        SHOTGUN_SPINUP,
        ROTATE_TRAY,
        WAIT_FOR_ROTATE,
        SHOOT,
        DONE
    }

    private final DarienOpModeFSM opMode;
    private final ShootArtifactFSM shootArtifactFSM;

    public ShootPatternFSM(DarienOpModeFSM opMode) {
        this.opMode = opMode;
        this.shootArtifactFSM = opMode.shootArtifactFSM;
    }

    private ArrayList<AprilTagDetection> aprilTagDetections;
    private Stage nbStep = Stage.IDLE;
    private int nbMotifIndex = 0;
    private double nbLastActionTime = 0;
    private boolean nbShootingActive = false;
    private double shootPower = 0;
    private boolean shotStarted = false;
    public static double TRAY_DELAY = 1.35;
    public static double SPINUP_DELAY = 0;

    public void startShootPattern(ArrayList<AprilTagDetection> detections, double currentTime, double shootingPower) {
        aprilTagDetections = detections;
        nbMotifIndex = 0;
        nbLastActionTime = currentTime;
        nbShootingActive = true;
        shootPower = shootingPower;
        shootArtifactFSM.setEjectionMotorsControlledByPattern(true);
        shootArtifactFSM.shotGun(shootPower);
        nbStep = Stage.SHOTGUN_SPINUP;
    }

    public void updateShootPattern(double currentTime) {
        if (!nbShootingActive) return;

        int switchselector = 0;

        if (aprilTagDetections == null || aprilTagDetections.isEmpty()) {
            // If the camera is not working or not detecting apriltags, go to default motif.
        } else {
            AprilTagDetection detection = aprilTagDetections.get(0); // Only use first detection for motif
            switchselector = detection.id;
        }

        switch (nbStep) {
            case IDLE:
                // nothing
                break;
            case SHOTGUN_SPINUP:
                if (currentTime - nbLastActionTime >= SPINUP_DELAY) {
                    nbLastActionTime = currentTime;
                }
                break;
            case SHOOT: // Shoot and wait for completion
                if (!shotStarted) {
                    shootArtifactFSM.startShooting(shootPower);
                    shotStarted = true;
                    nbLastActionTime = currentTime;
                }
                shootArtifactFSM.updateShooting();
                if (shootArtifactFSM.shootingDone() || currentTime - nbLastActionTime >= 2.0) {
                    shootArtifactFSM.resetShooting();
                    nbMotifIndex++;
                }
                break;
            case DONE:
                nbShootingActive = false;
                shootArtifactFSM.setEjectionMotorsControlledByPattern(false);
                break;
        }
    }

    public boolean isShootPatternDone() {
        return !nbShootingActive;
    }

}
