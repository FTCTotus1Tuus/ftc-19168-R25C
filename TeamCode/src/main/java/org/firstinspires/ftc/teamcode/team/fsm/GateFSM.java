package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class GateFSM {

    public enum GateStates {CLOSED, OPEN}

    // HARDWARE DEVICES
    private Servo gateServo;

    // HARDWARE TUNING CONSTANTS
    public static double GATE_OPEN = .4;
    public static double GATE_CLOSED = .55;

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
                close();
                break;
            case OPEN:
            default:
                open();
                break;
        }
    }

    /**
     * Gets the state
     *
     * @return Current state.
     */
    public GateStates getState() {
        return gateState;
    }

    /**
     * Sets the state
     *
     * @param state GateStates value
     */
    private void setState(GateStates state) {
        this.gateState = state;
    }

    /**
     * Request to open the gate
     */
    public void open() {
        gateServo.setPosition(GATE_OPEN);
        this.setState(GateStates.OPEN);
    }

    /**
     * Request to close the gate
     */
    public void close() {
        gateServo.setPosition(GATE_CLOSED);
        this.setState(GateStates.CLOSED);
    }

}
