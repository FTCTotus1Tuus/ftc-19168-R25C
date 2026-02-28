package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class IntakeFSM {
    public enum IntakeModes {OFF, FORWARD, REVERSE, FULL, SHOOT}

    public enum States {OFF, INTAKING, REVERSING, READYTOSHOOT}

    /**
     * Tracks detection state for a single distance sensor.
     * Each sensor gets its own instance so timing and flags are fully independent.
     * Junior devs: just call update() every loop and read isDetected() — that's it!
     */
    private static class SensorTracker {
        private final String sensorName;
        private double detectionStartTime = Double.NaN;
        private boolean isDetected = false;

        SensorTracker(String sensorName) {
            this.sensorName = sensorName;
        }

        /**
         * Call every loop. Requires DETECTION_HOLD_TIME seconds of continuous
         * detection before isDetected() returns true.
         */
        void update(double distanceCm, double currentTime, Telemetry telemetry) {
            if (distanceCm <= INTAKE_DISTANCE) {
                if (Double.isNaN(detectionStartTime)) {
                    detectionStartTime = currentTime;
                }
                if (currentTime - detectionStartTime >= DETECTION_HOLD_TIME) {
                    isDetected = true;
                    telemetry.addData(sensorName, "Detected");
                } else {
                    telemetry.addData(sensorName, "Detecting... %.1fs", currentTime - detectionStartTime);
                }
            } else {
                detectionStartTime = Double.NaN;
                isDetected = false;
                telemetry.addData(sensorName, "Not Detected");
            }
        }

        boolean isDetected() {
            return isDetected;
        }

        void reset() {
            detectionStartTime = Double.NaN;
            isDetected = false;
        }
    }

    // FSM DEPENDENCIES
    private final GateFSM gateFSM;

    // HARDWARE DEVICES
    private final DigitalChannel ledRightGreen, ledLeftGreen, ledRightRed, ledLeftRed;
    private final CRServo rampServoLow, rampServoHigh, rubberBandsMid, intakeRear;
    private final DcMotorEx rubberBandsFront;
    private final NormalizedColorSensor intakeColorSensor;
    private final NormalizedColorSensor middleColorSensor;
    private final NormalizedColorSensor turretColorSensor;

    // HARDWARE TUNING CONSTANTS
    public static double INTAKE_DISTANCE = 5; // cm
    public static double INTAKE_RUBBER_BANDS_POWER = .35;
    public static double INTAKE_RUBBER_BANDS_POWER_HIGH = 0.5;
    public static double OUTPUT_RUBBER_BANDS_POWER = 0.2;
    public static double INTAKE_INTAKE_ROLLER_POWER = 0.3;
    public static double INTAKE_INTAKE_ROLLER_POWER_HIGH = 1;
    public static double DETECTION_HOLD_TIME = 1.0; // seconds — how long a sensor must see an object before confirming

    // PRIVATE VARIABLES
    private IntakeModes mode = IntakeModes.OFF;
    private States state = States.OFF;

    // One tracker per sensor — each tracks its own timing and detected flag independently
    private final SensorTracker intakeSensorTracker;
    private final SensorTracker middleSensorTracker;
    private final SensorTracker turretSensorTracker;

    /**
     * Constructor
     *
     * @param hardwareMap Hardware map from the opmode
     * @param gateFSM     GateFSM dependency — IntakeFSM closes the gate when intaking
     */
    public IntakeFSM(HardwareMap hardwareMap, GateFSM gateFSM) {
        this.gateFSM = gateFSM;

        // INITIALIZE MOTORS
        rubberBandsFront = hardwareMap.get(DcMotorEx.class, "rubberBandsFront");

        // INITIALIZE SERVOS
        rampServoLow = hardwareMap.get(CRServo.class, "rampServoLow");
        rampServoHigh = hardwareMap.get(CRServo.class, "rampServoHigh");
        rubberBandsMid = hardwareMap.get(CRServo.class, "rubberBandsMid");
        intakeRear = hardwareMap.get(CRServo.class, "intakeRear");

        // INITIALIZE SENSORS
        intakeColorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeColorSensor");
        middleColorSensor = hardwareMap.get(NormalizedColorSensor.class, "middleColorSensor");
        turretColorSensor = hardwareMap.get(NormalizedColorSensor.class, "turretColorSensor");

        // INITIALIZE LEDs
        ledRightGreen = hardwareMap.get(DigitalChannel.class, "LEDRight1");
        ledLeftGreen = hardwareMap.get(DigitalChannel.class, "LEDLeft1");
        ledRightRed = hardwareMap.get(DigitalChannel.class, "LEDRight2");
        ledLeftRed = hardwareMap.get(DigitalChannel.class, "LEDLeft2");

        // INITIALIZE SENSOR TRACKERS — one per sensor, each fully independent
        intakeSensorTracker = new SensorTracker("intakeColorSensor");
        middleSensorTracker = new SensorTracker("middleColorSensor");
        turretSensorTracker = new SensorTracker("turretColorSensor");
    }

    /**
     * Initializes the hardware and state machine
     */
    public void init() {
        rubberBandsFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rubberBandsFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // Set LED channels to OUTPUT once — never needs to change
        ledLeftGreen.setMode(DigitalChannel.Mode.OUTPUT);
        ledLeftRed.setMode(DigitalChannel.Mode.OUTPUT);
        ledRightGreen.setMode(DigitalChannel.Mode.OUTPUT);
        ledRightRed.setMode(DigitalChannel.Mode.OUTPUT);
        setLedRed();
        mode = IntakeModes.OFF;
        state = States.OFF;
    }

    /**
     * Gets the current high-level state of the FSM.
     */
    public States getState() {
        return this.state;
    }

    /**
     * Gets the current motor/servo mode.
     */
    public IntakeModes getMode() {
        return this.mode;
    }

    private void setState(States state) {
        this.state = state;
    }

    /**
     * Resets all three sensor trackers to their default (not detected) state.
     */
    private void resetSensors() {
        intakeSensorTracker.reset();
        middleSensorTracker.reset();
        turretSensorTracker.reset();
    }

    // -------------------------------------------------------------------------
    // INTAKING STATE MACHINE
    // -------------------------------------------------------------------------

    /**
     * Call this to begin intaking. Sets motors forward and resets sensor state.
     */
    public void startIntaking() {
        setState(States.INTAKING);
        setMode(IntakeModes.FORWARD);
        resetSensors();
    }

    /**
     * Call this every loop while intaking.
     * Polls all three sensors and auto-transitions to FULL (motors stop pushing)
     * once all sensors have detected an object for DETECTION_HOLD_TIME seconds.
     *
     * @param currentTime getRuntime() from the OpMode
     * @param debug       true to emit extra telemetry
     * @param telemetry   Telemetry object
     */
    public void updateIntaking(double currentTime, boolean debug, Telemetry telemetry) {
        switch (state) {
            case INTAKING:
                setLedAmber();
                // Poll each sensor through its own independent tracker
                if (intakeColorSensor instanceof DistanceSensor) {
                    intakeSensorTracker.update(
                            ((DistanceSensor) intakeColorSensor).getDistance(DistanceUnit.CM),
                            currentTime, telemetry);
                }
                if (middleColorSensor instanceof DistanceSensor) {
                    middleSensorTracker.update(
                            ((DistanceSensor) middleColorSensor).getDistance(DistanceUnit.CM),
                            currentTime, telemetry);
                }
                if (turretColorSensor instanceof DistanceSensor) {
                    turretSensorTracker.update(
                            ((DistanceSensor) turretColorSensor).getDistance(DistanceUnit.CM),
                            currentTime, telemetry);
                }

                // As soon as the turret sensor sees the ball, cut intakeRear power so it
                // stops pushing the ball into the gate. The other rollers keep running.
                if (turretSensorTracker.isDetected()) {
                    intakeRear.setPower(0);
                    rampServoHigh.setPower(0);
                }

                boolean allDetected = intakeSensorTracker.isDetected()
                        && middleSensorTracker.isDetected()
                        && turretSensorTracker.isDetected();

                telemetry.addData("All Sensors Detected", allDetected);

                // Auto-stop: once the robot is full, switch to FULL mode (only mid-roller keeps running)
                if (mode == IntakeModes.FORWARD && allDetected) {
                    setLedGreen();
                    setMode(IntakeModes.FULL);
                    setState(States.READYTOSHOOT);
                }
                break;

            case READYTOSHOOT:
                mode = IntakeModes.FULL;
                setLedGreen();
                telemetry.addData("AUTO STOP", "All sensors detected — ready to shoot!");
                break;

            case REVERSING:
                mode = IntakeModes.REVERSE;
                break;

            case OFF:
            default:
                mode = IntakeModes.OFF;
                break;
        }
    }

    // -------------------------------------------------------------------------
    // PUBLIC CONTROL METHODS — Call these from TeleOpFSM
    // -------------------------------------------------------------------------

    /**
     * Reverse all rollers to eject artifacts.
     * No update loop needed — rollers stay reversed until off() or startIntaking() is called.
     */
    public void reverse() {
        setState(States.REVERSING);
        setMode(IntakeModes.REVERSE);
    }

    /**
     * Stop all intake motors.
     */
    public void off() {
        setState(States.OFF);
        setMode(IntakeModes.OFF);
    }

    /**
     * Returns true if all sensors have confirmed an object — robot is loaded and ready to shoot.
     */
    public boolean isReadyToShoot() {
        return state == States.READYTOSHOOT;
    }

    /**
     * Run all rollers at full SHOOT power to push the artifact through the gate.
     * Called by ShootingFSM when it opens the gate — IntakeFSM just handles the motors.
     */
    public void shootForward() {
        setMode(IntakeModes.SHOOT);
    }

    /**
     * Stop rollers and return to IDLE after a shot completes.
     * Called by ShootingFSM when it is done so intake is ready for the next cycle.
     */
    public void stopAfterShot() {
        setMode(IntakeModes.OFF);
        setState(States.OFF);
        setLedRed();
    }

    // ------------------------|
    // PRIVATE — MOTOR CONTROL |
    // ------------------------|

    private void setMode(IntakeModes mode) {
        this.mode = mode;
        switch (mode) {
            case SHOOT:
                rubberBandsFront.setPower(-INTAKE_RUBBER_BANDS_POWER_HIGH);
                rubberBandsMid.setPower(-INTAKE_INTAKE_ROLLER_POWER_HIGH);
                rampServoLow.setPower(INTAKE_INTAKE_ROLLER_POWER_HIGH);
                rampServoHigh.setPower(INTAKE_INTAKE_ROLLER_POWER_HIGH);
                intakeRear.setPower(-INTAKE_INTAKE_ROLLER_POWER_HIGH);
                break;
            case FORWARD:
                rubberBandsFront.setPower(-INTAKE_RUBBER_BANDS_POWER);
                rubberBandsMid.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                rampServoLow.setPower(INTAKE_INTAKE_ROLLER_POWER);
                rampServoHigh.setPower(INTAKE_INTAKE_ROLLER_POWER);
                intakeRear.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                gateFSM.close();
                break;
            case REVERSE:
                rubberBandsFront.setPower(OUTPUT_RUBBER_BANDS_POWER);
                rubberBandsMid.setPower(OUTPUT_RUBBER_BANDS_POWER);
                rampServoLow.setPower(-OUTPUT_RUBBER_BANDS_POWER);
                rampServoHigh.setPower(-OUTPUT_RUBBER_BANDS_POWER);
                intakeRear.setPower(OUTPUT_RUBBER_BANDS_POWER);
                break;
            case FULL:
                // Front rubber bands stop — only mid-roller keeps the artifact seated
                rubberBandsFront.setPower(-INTAKE_RUBBER_BANDS_POWER);
                rubberBandsMid.setPower(0);
                rampServoLow.setPower(0);
                rampServoHigh.setPower(0);
                intakeRear.setPower(0);
                break;
            case OFF:
            default:
                rubberBandsFront.setPower(0);
                rubberBandsMid.setPower(0);
                rampServoLow.setPower(0);
                rampServoHigh.setPower(0);
                intakeRear.setPower(0);
                break;
        }
    }

    // -------------------------------------------------------------------------
    // LED HELPERS
    // -------------------------------------------------------------------------

    public void setLedGreen() {
        ledLeftGreen.setState(false);
        ledLeftRed.setState(true);
        ledRightGreen.setState(false);
        ledRightRed.setState(true);
    }

    public void setLedRed() {
        ledLeftGreen.setState(true);
        ledLeftRed.setState(false);
        ledRightGreen.setState(true);
        ledRightRed.setState(false);
    }

    public void setLedAmber() {
        ledLeftGreen.setState(false);
        ledLeftRed.setState(false);
        ledRightGreen.setState(false);
        ledRightRed.setState(false);
    }

    public void setLedOff() {
        ledLeftGreen.setState(true);
        ledLeftRed.setState(true);
        ledRightGreen.setState(true);
        ledRightRed.setState(true);
    }
    
}
