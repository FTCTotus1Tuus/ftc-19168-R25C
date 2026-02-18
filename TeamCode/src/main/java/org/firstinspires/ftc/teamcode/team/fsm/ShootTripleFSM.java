package org.firstinspires.ftc.teamcode.team.fsm;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
public class ShootTripleFSM {

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

    public ShootTripleFSM(DarienOpModeFSM opMode) {
        this.opMode = opMode;
        this.shootArtifactFSM = opMode.shootArtifactFSM;
    }


    private Stage nbStep = Stage.IDLE;
    private int nbMotifIndex = 0;
    private double nbLastActionTime = 0;
    private boolean nbShootingActive = false;
    private double shootPower = 0;
    private boolean shotStarted = false;
    public static double TRAY_DELAY = 0.65;
    public static double SPINUP_DELAY = 0;
    //private ShootTripleFSM.Stage state = ShootTripleFSM.Stage.IDLE;
    /**
     * Returns true if the auto-intake state machine is currently running (positioning or intaking).
     */
    /*
    public boolean isRunning() {
        return state == ShootTripleFSM.Stage.SHOTGUN_SPINUP || state == ShootTripleFSM.Stage.ROTATE_TRAY || state == ShootTripleFSM.Stage.WAIT_FOR_ROTATE || state == ShootTripleFSM.Stage.SHOOT;
    }
     */

    public void startShootTriple(double currentTime, double shootingPower) {
        opMode.setLedAmber();
        nbMotifIndex = 0;
        nbShootingActive = true;
        shootPower = shootingPower;
        shootArtifactFSM.setEjectionMotorsControlledByPattern(true);
        shootArtifactFSM.shotGunTeleop(shootPower);
        nbLastActionTime = currentTime;
        nbStep = Stage.SHOTGUN_SPINUP;
    }

    public void updateShootTriple(double currentTime) {
        if (!nbShootingActive) return;

        int[] motif = null;
        // Motif order: 1=TRAY_POS_1_SCORE, 2=TRAY_POS_2_SCORE, 3=TRAY_POS_3_SCORE
        motif = new int[]{1, 2, 3};

        if (nbMotifIndex >= motif.length) {
            nbShootingActive = false;
            shootArtifactFSM.shotGunStop(); // Stop motors
            shootArtifactFSM.setEjectionMotorsControlledByPattern(false);
            return;
        }

        switch (nbStep) {
            case IDLE:
                // nothing
                break;
            case SHOTGUN_SPINUP:
                if (currentTime - nbLastActionTime >= SPINUP_DELAY) {
                    nbLastActionTime = currentTime;
                    nbStep = Stage.ROTATE_TRAY;
                }
                break;
            case ROTATE_TRAY: // Move tray
                double targetPos = (motif[nbMotifIndex] == 1) ? DarienOpModeFSM.TRAY_POS_1_SCORE :
                        (motif[nbMotifIndex] == 2) ? DarienOpModeFSM.TRAY_POS_2_SCORE :
                                DarienOpModeFSM.TRAY_POS_3_SCORE;
                //opMode.servoIncremental(opMode.TrayServo, targetPos, opMode.currentTrayPosition, 1, 4);
                //opMode.currentTrayPosition = targetPos;
                opMode.TrayServo.setPosition(targetPos);
                nbLastActionTime = currentTime;
                nbStep = Stage.WAIT_FOR_ROTATE;
                shotStarted = false; // Reset for next shot
                break;
            case WAIT_FOR_ROTATE: // Wait for tray move
                if (currentTime - nbLastActionTime >= TRAY_DELAY) {
                    nbLastActionTime = currentTime;
                    nbStep = Stage.SHOOT;
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
                    nbStep = Stage.ROTATE_TRAY;
                }
                break;
            case DONE:
                nbShootingActive = false;
                shootArtifactFSM.setEjectionMotorsControlledByPattern(false);
                break;
        }
        opMode.telemetry.addData("Actual ShotGun RPM", opMode.ejectionMotor.getVelocity() * 60 / DarienOpModeFSM.TICKS_PER_ROTATION); // convert from ticks per second to RPM
    }

    public boolean isDone() {
        return !nbShootingActive;
    }

    /*
    public void toggle() {
        if (()) {
            //stop();
        } else {
            startShootTriple(opMode.getRuntime(), );
        }
    }

     */

}
