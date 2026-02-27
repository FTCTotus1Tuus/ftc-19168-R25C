package org.firstinspires.ftc.teamcode.team.fsm;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class GateFSM {

    // HARDWARE DEVICES
    private Servo gateServo;

    // HARDWARE TUNING CONSTANTS
    public static double GATE_OPEN = .4;
    public static double GATE_CLOSED = .55;

    private enum GateStates {CLOSED, OPEN}

    private GateStates gateState = GateStates.CLOSED;

    /**
     * Constructor
     *
     * @param hardwareMap Hardwaremap from the opmode
     */
    public GateFSM(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, "gateServo");
    }

    /**
     * Initializes the hardware and state machine
     */
    public void init() {

    }

    /**
     * Updates the state machine and controls hardware based on the current state.
     *
     * @param currentTime Current time passed in as getRuntime()
     * @param debug       Boolean to control telemetry output
     * @param telemetry   Telemetry object
     */
    public void update(double currentTime, boolean debug, Telemetry telemetry) {
        switch (gateState) {
            case CLOSED:
                gateServo.setPosition(GATE_CLOSED);
                break;
            case OPEN:
            default:
                gateServo.setPosition(GATE_OPEN);
                break;
        }
    }

    /**
     * Gets the state
     *
     * @return Current state as a string.
     */
    public String getState() {
        return gateState.toString();
    }

    /**
     * Request to open the gate
     */
    public void open() {
        gateState = GateStates.OPEN;
    }

    /**
     * Request to close the gate
     */
    public void close() {
        gateState = GateStates.CLOSED;
    }

}
