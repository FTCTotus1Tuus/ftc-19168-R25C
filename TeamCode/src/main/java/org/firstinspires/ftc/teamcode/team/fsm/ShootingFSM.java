package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * ShootingFSM — owns the complete single-artifact shoot sequence.
 * <p>
 * Responsibilities:
 * - Spin up the ejection motor (via ShotgunFSM)
 * - Open and close the gate (via GateFSM) with configurable timing
 * - Stop the ejection motor when done
 * - Expose a clean start / update / query API to TeleOpFSM and autonomous OpModes
 * <p>
 * NOT responsible for:
 * - Intake roller control  → IntakeFSM
 * - Turret aiming          → TurretFSM
 * - Multi-artifact patterns → ShootPatternFSM
 * <p>
 * Typical usage (TeleOp):
 * // On button press:
 * shootingFSM.start(getRuntime(), ShootingFSM.PowerLevel.CLOSE);
 * <p>
 * // Every loop:
 * shootingFSM.update(getRuntime(), telemetry);
 * <p>
 * // When finished, let intake resume:
 * if (shootingFSM.isDone()) { intakeFSM.startIntaking(); }
 */
@Config
@Configurable
public class ShootingFSM {

    // -------------------------------------------------------------------------
    // ENUMS
    // -------------------------------------------------------------------------

    /**
     * High-level stage of the shooting sequence.
     * Junior devs: read this to know what the robot is doing, but never set it directly.
     */
    public enum Stage {
        IDLE,        // waiting for start()
        SPINNING_UP, // ejection motor spinning up to target RPM before opening gate
        OPENING_GATE,   // gate is open — artifact is passing through
        SHOOTING,
        START_CLOSING_GATE,
        CLOSING_GATE,  // gate has closed — brief settle before declaring done
        DONE         // sequence complete, motor still running (caller decides when to stop)
    }

    /**
     * Shooting power preset, selected by the driver or autonomous routine.
     * Maps to the RPM constants defined in DarienOpModeFSM.
     */
    public enum PowerLevel {
        CLOSE, // standard distance — SHOT_GUN_POWER_UP_RPM
        FAR    // long distance — SHOT_GUN_POWER_UP_FAR_RPM_TELEOP
    }

    // -------------------------------------------------------------------------
    // DEPENDENCIES
    // -------------------------------------------------------------------------

    private final GateFSM gateFSM;
    private final ShotgunFSM shotgunFSM;
    private final IntakeFSM intakeFSM;
    private final DarienOpModeFSM parent;

    // -------------------------------------------------------------------------
    // TUNING CONSTANTS  (visible in FTC Dashboard)
    // -------------------------------------------------------------------------

    public static double SPINUP_DELAY = 0.2;  // seconds to wait for motor to reach speed before opening gate
    public static double GATE_OPEN_DELAY = 0.35; // seconds to hold gate open
    public static double GATE_CLOSE_DELAY = 0.1; // seconds to settle after gate closes

    // -------------------------------------------------------------------------
    // PRIVATE STATE
    // -------------------------------------------------------------------------

    private Stage stage = Stage.IDLE;
    private double stageStartTime = Double.NaN;

    // -------------------------------------------------------------------------
    // CONSTRUCTOR
    // -------------------------------------------------------------------------

    /**
     * @param gateFSM    Gate dependency — ShootingFSM calls open() and close()
     * @param shotgunFSM Ejection motor dependency — ShootingFSM calls toPowerUp/toPowerUpFar/toOff
     * @param intakeFSM  Intake dependency — ShootingFSM calls shootForward() and stopAfterShot()
     * @param parent     Parent OpMode reference for accessing shooting power mode and odometry
     */
    public ShootingFSM(GateFSM gateFSM, ShotgunFSM shotgunFSM, IntakeFSM intakeFSM, DarienOpModeFSM parent) {
        this.gateFSM = gateFSM;
        this.shotgunFSM = shotgunFSM;
        this.intakeFSM = intakeFSM;
        this.parent = parent;
    }

    // -------------------------------------------------------------------------
    // PUBLIC API
    // -------------------------------------------------------------------------

    /**
     * Begin the shoot sequence.
     * Spins up the ejection motor at the chosen power level, then opens the gate.
     *
     * @param currentTime getRuntime() from the OpMode
     * @param powerLevel  CLOSE or FAR — selects the target RPM
     */
    public void start(double currentTime, PowerLevel powerLevel) {
        spinUp(powerLevel);
        stage = Stage.SPINNING_UP;
        stageStartTime = currentTime;
    }

    public void finish() {
        stage = Stage.START_CLOSING_GATE;
    }

    /**
     * Call every loop after start().
     * Drives: SPINNING_UP → GATE_OPEN → GATE_CLOSE → DONE
     * When DONE, the ejection motor is left running so the caller can decide
     * whether to stop it immediately or hold it for a follow-up shot.
     *
     * @param currentTime getRuntime() from the OpMode
     * @param telemetry   Telemetry object for debug output
     */
    public void update(double currentTime, Telemetry telemetry) {
        double elapsed = currentTime - stageStartTime;

        switch (stage) {
            case SPINNING_UP:
                intakeFSM.setLedAmber();
                telemetry.addData("SHOOTING", "Spinning up — %.2fs / %.2fs", elapsed, SPINUP_DELAY);
                if (elapsed >= SPINUP_DELAY) {
                    gateFSM.open();
                    stage = Stage.OPENING_GATE;
                    stageStartTime = currentTime;
                }
                break;

            case OPENING_GATE:
                telemetry.addData("SHOOTING", "Gate open — %.2fs / %.2fs", elapsed, GATE_OPEN_DELAY);
                if (elapsed >= GATE_OPEN_DELAY) {
                    intakeFSM.shootForward(); // run rollers at SHOOT power as gate opens
                    stage = Stage.SHOOTING;
                    stageStartTime = currentTime;
                }
                break;

            case SHOOTING:
                //do nothing
                break;

            case START_CLOSING_GATE:
                gateFSM.close();
                stage = Stage.CLOSING_GATE;
                break;

            case CLOSING_GATE:
                telemetry.addData("SHOOTING", "Gate closing — %.2fs / %.2fs", elapsed, GATE_CLOSE_DELAY);
                if (elapsed >= GATE_CLOSE_DELAY) {
                    intakeFSM.stopAfterShot(); // stop rollers, LED red, intake back to IDLE
                    stage = Stage.DONE;
                }
                break;

            case DONE:
                telemetry.addData("SHOOTING", "Done");
                break;

            case IDLE:
            default:
                break;
        }
    }

    /**
     * Returns true once the full spin-up → gate open → gate close sequence has completed.
     * Use this to know when the robot is safe to start intaking again.
     */
    public boolean isDone() {
        return stage == Stage.DONE;
    }

    public boolean isBusy() {
        return stage != Stage.DONE || stage != Stage.IDLE;
    }

    /**
     * Returns the current stage — useful for telemetry and autonomous logic.
     */
    public Stage getStage() {
        return stage;
    }

    /**
     * Stop the ejection motor and reset back to IDLE.
     * Call this after isDone(), or to abort a shot in progress.
     */
    public void reset() {
        shotgunFSM.toOff();
        stage = Stage.IDLE;
        stageStartTime = Double.NaN;
    }

    /**
     * Convenience: stop motor and return to IDLE without affecting gate state.
     * Useful when aborting due to timeout.
     */
    public void abort() {
        gateFSM.close();
        reset();
    }

    // -------------------------------------------------------------------------
    // PRIVATE HELPERS
    // -------------------------------------------------------------------------

    private void spinUp(PowerLevel powerLevel) {
        // Determine effective power level (may be overridden by odometry)
        PowerLevel effectivePowerLevel = powerLevel;

        // If in ODOMETRY mode, override power level based on robot Y position
        if (parent.shootingPowerMode == DarienOpModeFSM.ShootingPowerModes.ODOMETRY) {
            double robotY = parent.getRobotY();
            if (!Double.isNaN(robotY)) {
                // FAR power if Y <= threshold (closer to audience side), CLOSE otherwise
                effectivePowerLevel = (robotY <= DarienOpModeFSM.SHOOTING_POWER_ODOMETRY_Y_THRESHOLD)
                    ? PowerLevel.FAR
                    : PowerLevel.CLOSE;
            }
        }

        // Select appropriate RPM constant based on power level and autonomous/teleop mode
        double targetRPM;
        if (effectivePowerLevel == PowerLevel.FAR) {
            targetRPM = parent.isAutonomousMode()
                ? DarienOpModeFSM.SHOT_GUN_POWER_UP_FAR_RPM_AUTO
                : DarienOpModeFSM.SHOT_GUN_POWER_UP_FAR_RPM_TELEOP;
            shotgunFSM.toPowerUpFar(targetRPM);
        } else {
            targetRPM = parent.isAutonomousMode()
                ? DarienOpModeFSM.SHOT_GUN_POWER_UP_RPM_AUTO
                : DarienOpModeFSM.SHOT_GUN_POWER_UP_RPM;
            shotgunFSM.toPowerUp(targetRPM);
        }
    }
}









