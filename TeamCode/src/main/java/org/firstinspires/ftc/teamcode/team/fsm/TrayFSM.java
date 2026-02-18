package org.firstinspires.ftc.teamcode.team.fsm;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

/**
 * State machine to automate the tray intake.
 * Usage:
 * - Construct with your hardware objects (tray servo, two intake CRServos, color sensor, telemetry).
 * - Call startAutoIntake() to begin the state machine. Periodically call update() from loop().
 * - Telemetry will show raw RGB (0-255) and detected slot states.
 */
@Config
public class TrayFSM {
    public enum SlotState { EMPTY, PURPLE, GREEN, UNKNOWN }
    private enum State { IDLE, POSITION_TO_SLOT, INTAKE_WAIT, CHECK_SLOT, FINISHING, DONE }

    private final DarienOpModeFSM opMode;

    private final Servo trayServo;
    private final DcMotorEx rubberBands;      // Motor for rubber band intake
    private final CRServo topIntake;       // CRServo for top intake
    private final CRServo rightIntake;
    private final CRServo leftIntake;
    private final NormalizedColorSensor colorSensor;
    private final Telemetry telemetry;
    private final ElapsedTime timer = new ElapsedTime();
    private final DigitalChannel ledRight1;
    private final DigitalChannel ledLeft1;
    private final DigitalChannel ledRight2;
    private final DigitalChannel ledLeft2;

    // Slot servo target positions for intake slots (0, 1, 2 correspond to tray intake positions 3, 1, 2)
    private final double[] slotPositions = new double[]{DarienOpModeFSM.TRAY_POS_3_INTAKE, DarienOpModeFSM.TRAY_POS_1_INTAKE, DarienOpModeFSM.TRAY_POS_2_INTAKE};


    // Internal state
    private final SlotState[] slots = new SlotState[3];
    private State state = State.IDLE;
    private int currentSlotIndex = 0; // 0..2

    // Timing/timeouts (in seconds)
    public static double INTAKE_TIMEOUT = 2.0; // how long to run intake while waiting for ball (used only if waitForArtifact==false)
    public static double TRAY_DELAY = 0.5;    // how long to wait after moving servo before checking
    // How long to ignore sensor input after commanding the servo to move (seconds).
    // Set this slightly longer than the servo travel time to avoid seeing balls that pass under the sensor during rotation.
    public static double SERVO_IGNORE_DURATION = TRAY_DELAY;

    // Whether to wait indefinitely for an artifact (true) or use timeout to skip (false)
    public static boolean WAIT_FOR_ARTIFACT = true;

    // Color detection thresholds (RGB and HSV)
    public static int PRESENCE_THRESHOLD = 1; // brightness threshold to decide something is present (0..255). Lowered to handle low readings like 1/2/3
    public static int PURPLE_THRESHOLD = 40;   // RGB fallback threshold for purple
    public static int GREEN_THRESHOLD = 40;    // RGB fallback threshold for green

    // HSV parameters (hue in degrees 0..360, sat/val 0..1)
    public static float GREEN_HUE_MIN = 120f;
    public static float GREEN_HUE_MAX = 180f;
    public static float PURPLE_HUE_MIN = 180f;
    public static float PURPLE_HUE_MAX = 240f;
    public static float SAT_THRESHOLD = 0.25f;
    public static float VAL_THRESHOLD = 0.006f; // lower default to handle low-light readings

    // Low-light relative detection (dashboard-tunable)
    // When readings are extremely small (e.g. RGB 1/2/3), absolute HSV/RGB thresholds may fail.
    // Enabling this causes the classifier to use the normalized component ratios to infer color.
    public static boolean ALLOW_LOW_LIGHT_DETECTION = true;
    // The dominant normalized component (or sum of red+blue for purple) must exceed others by this factor
    public static float LOW_LIGHT_RELATIVE_FACTOR = 1.1f;
    // Minimum total normalized intensity (red+green+blue) to consider the relative heuristic
    public static float LOW_LIGHT_MIN_TOTAL = 0.005f;

    // After a ball is detected and the intake motors stop, wait this many seconds for the ball to settle
    // into the tray slot before rotating to the next slot. Tunable via Dashboard.
    public static double BALL_SETTLE_TIME = 0.1;

    // Sliding-window detection
    private final int windowSize = 10; // number of recent samples to consider
    private final SlotState[] detectionWindow = new SlotState[windowSize];
    private int windowIndex = 0;
    public static int REQUIRED_DETECTIONS = 5; // how many occurrences in window to accept

    // Brightness gating (optional). If >0, require brightness increase over baseline to count samples
    private int baselineBrightness = 0;
    public static int BRIGHTNESS_DELTA_THRESHOLD = 0; // disabled by default

    // Tracking
    private double stateStartTime = 0.0;
    // Timestamp (seconds) until which sensor input should be ignored because the servo is still moving
    private double servoIgnoreUntil = 0.0;

    // Constructor: initialize final hardware fields and detection window
    public TrayFSM(DarienOpModeFSM opMode, Servo trayServo, DcMotorEx rubberBands, CRServo topIntake, CRServo rightIntake, CRServo leftIntake, NormalizedColorSensor colorSensor, Telemetry telemetry, DigitalChannel ledRight1, DigitalChannel ledLeft1, DigitalChannel ledRight2, DigitalChannel ledLeft2) {
        this.opMode = opMode;
        this.trayServo = trayServo;
        this.rubberBands = rubberBands;
        this.topIntake = topIntake;
        this.rightIntake = rightIntake;
        this.leftIntake = leftIntake;
        this.colorSensor = colorSensor;
        this.telemetry = telemetry;
        this.ledRight1 = ledRight1;
        this.ledLeft1 = ledLeft1;
        this.ledRight2 = ledRight2;
        this.ledLeft2 = ledLeft2;

        // initialize slot states and detection window
        Arrays.fill(slots, SlotState.EMPTY);
        Arrays.fill(detectionWindow, SlotState.EMPTY);
    }

    // Dashboard-editable intake powers
    public static double INTAKE_RUBBER_BANDS_POWER = DarienOpModeFSM.INTAKE_RUBBER_BANDS_POWER;
    public static double INTAKE_INTAKE_ROLLER_POWER = DarienOpModeFSM.INTAKE_INTAKE_ROLLER_POWER;

    // Start the automatic intake process. This will clear previous slot data and begin at first empty intake slot.
    public void startAutoIntake() {
        opMode.setLedRed();
        Arrays.fill(slots, SlotState.EMPTY);
        // reset detection window so previous readings don't affect new intake
        Arrays.fill(detectionWindow, SlotState.EMPTY);
        windowIndex = 0;
        currentSlotIndex = findNextEmptySlot(0);
        if (currentSlotIndex < 0) {
            state = State.FINISHING;
            return;
        }
        state = State.POSITION_TO_SLOT;
        stateStartTime = timer.seconds();
        moveToSlot(currentSlotIndex);
     }

    // Call this periodically (e.g., opmode loop)
    public void update() {
        // If the color sensor isn't configured, avoid NPE and report via telemetry
        if (colorSensor == null) {
            telemetry.addLine("TrayFSM: intakeColorSensor not configured");
            // Output slot summary
            for (int i = 0; i < 3; i++) {
                telemetry.addData("Slot " + (i + 1), slots[i].toString());
            }
            //telemetry.update();
            return;
        }

        // Read RGB scaled to 0-255
        int[] rgb = readRGB255();
        // Add telemetry for raw rgb (0-255)
        telemetry.addData("RGB", "%d / %d / %d", rgb[0], rgb[1], rgb[2]);
        float[] hsv = new float[3];
        Color.RGBToHSV(rgb[0], rgb[1], rgb[2], hsv); // hsv[0]=hue(0..360), hsv[1]=sat(0..1), hsv[2]=val(0..1)
        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];
        telemetry.addData("Hue", "%.1f", hue);
        telemetry.addData("Sat/Val", "%.3f/%.3f", sat, val);

        switch (state) {
            case IDLE:
                // nothing
                break;

            case POSITION_TO_SLOT:
                // Wait a short settle time after moving servo
                // only proceed to INTAKE_WAIT after both settle time and servo movement are finished
                if (timer.seconds() - stateStartTime >= TRAY_DELAY && timer.seconds() >= servoIgnoreUntil) {
                    // Start intake to try to capture a ball: run both rubber bands and the roller
                    rubberBands.setPower(INTAKE_RUBBER_BANDS_POWER);
                    topIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                    //rightIntake.setPower(INTAKE_INTAKE_ROLLER_POWER);
                    //leftIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                    // reset detection window for this intake slot
                    Arrays.fill(detectionWindow, SlotState.EMPTY);
                     windowIndex = 0;
                     // capture baseline brightness so we only accept arrivals that increase brightness
                     int[] baselineRgb = readRGB255();
                     baselineBrightness = (baselineRgb[0] + baselineRgb[1] + baselineRgb[2]) / 3;
                     // reset detection window for this intake slot
                     state = State.INTAKE_WAIT;
                     stateStartTime = timer.seconds();
                     telemetry.addData("State", "INTAKE_WAIT slot=%d", currentSlotIndex + 1);
                 }
                 break;

            case INTAKE_WAIT:
                // while intake running, monitor sensor to see if a ball has arrived
                SlotState detected = classifyCurrentColor();
                int[] currRgb = readRGB255();
                int currBrightness = (currRgb[0] + currRgb[1] + currRgb[2]) / 3;
                telemetry.addData("DetectedRaw", detected.toString());
                telemetry.addData("Brightness", "%d (baseline %d, delta %d)", currBrightness, baselineBrightness, BRIGHTNESS_DELTA_THRESHOLD);

                // Put this detection into the sliding window (apply brightness gating if configured)
                boolean countThisSample = true;
                if (BRIGHTNESS_DELTA_THRESHOLD > 0) {
                    countThisSample = (currBrightness - baselineBrightness) >= BRIGHTNESS_DELTA_THRESHOLD;
                }
                if (!countThisSample) {
                    detected = SlotState.EMPTY; // treat as empty for counting purposes
                }

                // push into window
                // If the servo is still moving / ignoring period hasn't passed, don't count this sample
                if (timer.seconds() >= servoIgnoreUntil) {
                    detectionWindow[windowIndex] = detected;
                    windowIndex = (windowIndex + 1) % windowSize;
                } else {
                    // treat as empty sample while servo is settling
                    detectionWindow[windowIndex] = SlotState.EMPTY;
                    windowIndex = (windowIndex + 1) % windowSize;
                }

                // count occurrences in window
                int countPurple = 0;
                int countGreen = 0;
                for (int i = 0; i < windowSize; i++) {
                    if (detectionWindow[i] == SlotState.PURPLE) countPurple++;
                    if (detectionWindow[i] == SlotState.GREEN) countGreen++;
                }

                // Manual overrides for testing (gamepad2 X=purple, Y=green)
                if (opMode.gamepad1.dpad_left) {
                    countPurple += REQUIRED_DETECTIONS;
                } if (opMode.gamepad1.dpad_right) {
                countGreen += REQUIRED_DETECTIONS;
                }

                telemetry.addData("WindowCounts", "P=%d G=%d", countPurple, countGreen);

                // Determine if either color meets the required detections
                if (countPurple >= REQUIRED_DETECTIONS || countGreen >= REQUIRED_DETECTIONS) {
                    SlotState accepted = (countPurple >= countGreen) ? SlotState.PURPLE : SlotState.GREEN;
                    // Ball detected reliably — stop intake and record it, but wait before rotating so the
                    // ball can settle into the slot.
                    //rubberBands.setPower(0.0);
                    //topIntake.setPower(0.0);
                    //leftIntake.setPower(0.0);
                    //rightIntake.setPower(0.0);
                    slots[currentSlotIndex] = accepted;
                    telemetry.addData("Slot " + (currentSlotIndex + 1), slots[currentSlotIndex].toString());
                    // Determine next empty slot index. We will move there after BALL_SETTLE_TIME.
                    int next = findNextEmptySlot(currentSlotIndex + 1);
                    if (next < 0) {
                        state = State.FINISHING;
                    } else {
                        currentSlotIndex = next;
                        // enter settling state; CHECK_SLOT will handle the post-detection delay
                        state = State.CHECK_SLOT;
                        stateStartTime = timer.seconds();
                        telemetry.addData("State", "POST_DETECT_SETTLE slot=%d", currentSlotIndex + 1);
                    }
                } else if (!WAIT_FOR_ARTIFACT && timer.seconds() - stateStartTime >= INTAKE_TIMEOUT) {
                     // Timeout behavior only used when waitForArtifact is false.
                     // Timeout, assume no ball arrived. Stop intake and mark EMPTY, go to next slot
                    //rubberBands.setPower(0.0);
                    //topIntake.setPower(0.0);
                    //leftIntake.setPower(0.0);
                    //rightIntake.setPower(0.0);
                     slots[currentSlotIndex] = SlotState.EMPTY;
                     int next = findNextEmptySlot(currentSlotIndex + 1);
                     if (next < 0) {
                         state = State.FINISHING;
                     } else {
                         currentSlotIndex = next;
                         moveToSlot(currentSlotIndex);
                         state = State.POSITION_TO_SLOT;
                         stateStartTime = timer.seconds();
                     }
                 } else {
                     telemetry.addData("IntakeWaiting", "slot=%d time=%.2f", currentSlotIndex + 1, timer.seconds() - stateStartTime);
                 }
                 break;

            case CHECK_SLOT:
                // Post-detection settle: wait BALL_SETTLE_TIME to allow the ball to settle in the tray
                if (timer.seconds() - stateStartTime >= BALL_SETTLE_TIME) {
                    // After settling, move to the next empty intake slot
                    moveToSlot(currentSlotIndex);
                    state = State.POSITION_TO_SLOT;
                    stateStartTime = timer.seconds();
                    telemetry.addData("State", "POSITION_TO_SLOT after settle slot=%d", currentSlotIndex + 1);
                } else {
                    telemetry.addData("State", "SETTLING for %.2fs", BALL_SETTLE_TIME - (timer.seconds() - stateStartTime));
                }
                break;
            case FINISHING:
                opMode.TrayServo.setPosition(DarienOpModeFSM.TRAY_POS_1_SCORE);
                state = State.DONE;
                break;

            case DONE:
                telemetry.addData("TrayFSM", "DONE");
                // leave motors off
                rubberBands.setPower(0.0);
                topIntake.setPower(0.0);
                //rightIntake.setPower(0.0);
                //leftIntake.setPower(0.0);
                break;
        }

        // Output slot summary
        for (int i = 0; i < 3; i++) {
            telemetry.addData("Slot " + (i + 1), slots[i].toString());
        }

        // Telemetry: show if we're currently ignoring sensor input due to servo movement
        double ignoreRemaining = Math.max(0.0, servoIgnoreUntil - timer.seconds());
        telemetry.addData("ServoIgnoreRemaining", "%.2fs", ignoreRemaining);
        telemetry.addData("IgnoringSensorInput", "%s", ignoreRemaining > 0.0 ? "YES" : "NO");

         //telemetry.update();
     }

    // Move servo to the given intake slot index (0..2)
    private void moveToSlot(int slotIndex) {
        double pos = slotPositions[Math.max(0, Math.min(2, slotIndex))];
        trayServo.setPosition(pos);
        // ignore sensor input for a short time while the servo moves
        servoIgnoreUntil = timer.seconds() + SERVO_IGNORE_DURATION;
     }

    // Move servo to a scoring position index (0..2) corresponding to scoring positions 1..3
    public void moveToScoreSlot(int scoreSlotIndex) {
        int i = Math.max(0, Math.min(2, scoreSlotIndex));
        double pos;
        switch (i) {
            case 0: pos = DarienOpModeFSM.TRAY_POS_3_SCORE; break;
            case 1: pos = DarienOpModeFSM.TRAY_POS_2_SCORE; break;
            default: pos = DarienOpModeFSM.TRAY_POS_1_SCORE; break;
        }
        trayServo.setPosition(pos);
    }

    // Returns next index >= start that is considered empty in the internal slots array; -1 if none
    private int findNextEmptySlot(int start) {
        for (int i = start; i < 3; i++) {
            if (slots[i] == SlotState.EMPTY) return i;
        }
        return -1;
    }

    // Read color sensor and return RGB scaled to 0..255
    private int[] readRGB255() {
        // NormalizedColorSensor returns floats 0..1. Convert to 0..255 and clamp.
        com.qualcomm.robotcore.hardware.NormalizedRGBA colors = colorSensor.getNormalizedColors();
        int r = normTo255(colors.red);
        int g = normTo255(colors.green);
        int b = normTo255(colors.blue);
        return new int[]{r, g, b};
    }

    private int normTo255(float v) {
        if (Float.isNaN(v)) return 0;
        float clamped = Math.max(0f, Math.min(1f, v));
        return (int) (clamped * 255f + 0.5f);
    }

    // Classify current color reading into EMPTY / PURPLE / GREEN / UNKNOWN
    private SlotState classifyCurrentColor() {
        // Read normalized colors (0..1) and convert to 0..255 for logging and HSV conversion
        com.qualcomm.robotcore.hardware.NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float nr = colors.red;
        float ng = colors.green;
        float nb = colors.blue;
        int r = normTo255(nr);
        int g = normTo255(ng);
        int b = normTo255(nb);
        int brightness = (r + g + b) / 3;

        // Telemetry: always show normalized floats and converted RGB so you can tune thresholds live
        telemetry.addData("Norm R/G/B", "%.4f / %.4f / %.4f", nr, ng, nb);
        telemetry.addData("RGB 0-255", "%d / %d / %d", r, g, b);
        telemetry.addData("BrightnessRaw", "%d", brightness);

        // Convert RGB (0-255) to HSV and always emit HSV telemetry even at low brightness
        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv); // hsv[0]=hue(0..360), hsv[1]=sat(0..1), hsv[2]=val(0..1)
        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];
        telemetry.addData("Hue", "%.1f", hue);
        telemetry.addData("Sat/Val", "%.3f/%.3f", sat, val);

        // If very dark, optionally try a relative, normalized-component heuristic
        boolean lowBrightness = brightness < PRESENCE_THRESHOLD;
        boolean lowValue = val < VAL_THRESHOLD;

        /*
        if (lowBrightness) {
            // If brightness is below the presence threshold, we won't accept HSV absolute criteria.
            // Instead allow the low-light relative heuristic (if enabled) to classify by ratios.
            if (ALLOW_LOW_LIGHT_DETECTION) {
                float total = nr + ng + nb; // normalized total (0..3)
                if (total >= LOW_LIGHT_MIN_TOTAL) {
                    if (ng > (nr + nb) * LOW_LIGHT_RELATIVE_FACTOR) return SlotState.GREEN;
                    if ((nr + nb) > ng * LOW_LIGHT_RELATIVE_FACTOR && (nr + nb) > 0f) return SlotState.PURPLE;
                }
            }
            return SlotState.EMPTY;
        } else if (ALLOW_LOW_LIGHT_DETECTION && lowValue) {
            // If HSV value is too low for reliable hue, still try the relative heuristic
            float total = nr + ng + nb;
            if (total >= LOW_LIGHT_MIN_TOTAL) {
                if (ng > (nr + nb) * LOW_LIGHT_RELATIVE_FACTOR) return SlotState.GREEN;
                if ((nr + nb) > ng * LOW_LIGHT_RELATIVE_FACTOR && (nr + nb) > 0f) return SlotState.PURPLE;
            }
        }

         */

         // Use HSV ranges first (more robust to lighting)
        //if (val >= VAL_THRESHOLD && sat >= SAT_THRESHOLD) {
            if (isHueInRange(hue, GREEN_HUE_MIN, GREEN_HUE_MAX)) return SlotState.GREEN;
            if (isHueInRange(hue, PURPLE_HUE_MIN, PURPLE_HUE_MAX)) return SlotState.PURPLE;
        //}


         // Fallback to RGB heuristics
        /*
         if (g >= r && g >= b && g >= GREEN_THRESHOLD) return SlotState.GREEN;
         int redBlueAvg = (r + b) / 2;
         if (redBlueAvg >= PURPLE_THRESHOLD && redBlueAvg > g) return SlotState.PURPLE;

         */
         return SlotState.UNKNOWN;

    }



    private boolean isHueInRange(float hue, float min, float max) {
        if (min <= max) return hue >= min && hue <= max;
        // wrap-around case (e.g., min=300, max=40)
        return hue >= min || hue <= max;
    }

    // Public getters
    public SlotState[] getSlots() {
        return slots.clone();
    }

    public SlotState getSlot(int index) {
        if (index < 0 || index >= slots.length) return SlotState.UNKNOWN;
        return slots[index];
    }

    public boolean isDone() { return state == State.DONE; }

    public void stop() {
        rubberBands.setPower(0);
        topIntake.setPower(0.0);
        state = State.IDLE;
    }

    /**
     * Returns true if the auto-intake state machine is currently running (positioning or intaking).
     */
    public boolean isAutoIntakeRunning() {
        return state == State.POSITION_TO_SLOT || state == State.INTAKE_WAIT || state == State.CHECK_SLOT;
    }

    /**
     * Stop the auto-intake state machine and ensure intake motors are off.
     */
    public void stopAutoIntake() {
        rubberBands.setPower(0.0);
        topIntake.setPower(0.0);
        state = State.FINISHING;
    }

    /**
     * Forcefully stop the auto-intake immediately.
     * Stops motors, clears detection buffer, resets servo ignore timers, and sets FSM to IDLE
     * so it can be restarted cleanly with startAutoIntake().
     */
    public void forceStopAutoIntake() {
        // stop motors immediately
        rubberBands.setPower(0.0);
        topIntake.setPower(0.0);
        // clear detection buffer and reset window index
        Arrays.fill(detectionWindow, SlotState.EMPTY);
        windowIndex = 0;
        // reset servo-ignore timer so sensor sampling resumes immediately once restarted
        servoIgnoreUntil = 0.0;
        // set FSM to IDLE to prevent further state transitions until user restarts
        state = State.IDLE;
        // optional telemetry feedback
        if (telemetry != null) {
            telemetry.addData("TrayFSM", "FORCE_STOP");
            //telemetry.update();
        }
    }

    /**
     * Toggle the auto-intake state machine: start if stopped, stop if running.
     */
    public void toggleAutoIntake() {
        if (isAutoIntakeRunning()) {
            stopAutoIntake();
        } else {
            startAutoIntake();
        }
    }

}
