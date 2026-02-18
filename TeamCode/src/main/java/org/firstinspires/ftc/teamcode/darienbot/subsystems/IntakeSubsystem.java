package org.firstinspires.ftc.teamcode.darienbot.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {

    // HARDWARE COMPONENTS
    private final Servo intake;
    private CRServo rubberBands;

    // HARDWARE TUNING CONSTANTS
    public static double INTAKE_SERVO_POS_UP = 0.75;
    public static double INTAKE_SERVO_POS_DOWN = 0.21;
    public static double INTAKE_DISTANCE = 5;//in CM
    public static double INTAKE_TIME = 1;
    public static double INTAKE_RUBBER_BANDS_POWER = -0.7;
    public static double OUTPUT_RUBBER_BANDS_POWER = 0.2;

    // HARDWARE CONTROLS
    double IntakeServoPosition = 0;

    public IntakeSubsystem(HardwareMap hw) {
        intake = hw.get(Servo.class, "intakeServo");
        rubberBands = hw.get(CRServo.class, "rubberBands");
    }

    public void update() {
    }

    public void stop() {
        intake.setPosition(INTAKE_SERVO_POS_DOWN);
        rubberBands.setPower(0);
    }

}
