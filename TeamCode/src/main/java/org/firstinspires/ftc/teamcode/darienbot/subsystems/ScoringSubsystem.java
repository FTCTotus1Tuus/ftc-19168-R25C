package org.firstinspires.ftc.teamcode.darienbot.subsystems;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.darienbot.util.DarienUtil;

public class ScoringSubsystem {

    // HARDWARE COMPONENTS
    public DcMotor ejectionMotorRight;
    public DcMotor ejectionMotorLeft;
    public Servo Elevator;
    public Servo Feeder;

    // HARDWARE TUNING CONSTANTS
    public static double ELEVATOR_POS_UP = 0.83;
    public static double ELEVATOR_POS_DOWN = 0.45;
    public static double FEEDER_POS_UP = .9;
    public static double FEEDER_POS_DOWN = .45;
    public static double SHOT_GUN_POWER_UP = 1;

    // HELPERS
    public DarienUtil darienUtil;

    public ScoringSubsystem(HardwareMap hw) {
        // Initialize ejection motors
        //ejectionMotorLeft = darienUtil.initializeMotor("ejectionMotorLeft", hw);
        //ejectionMotorRight = darienUtil.initializeMotor("ejectionMotorRight", hw);
        ejectionMotorLeft = hw.get(DcMotor.class, "ejectionMotorLeft");
        ejectionMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ejectionMotorRight = hw.get(DcMotor.class, "ejectionMotorRight");
        ejectionMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Optionally set motor directions or modes
        ejectionMotorRight.setDirection(DcMotor.Direction.FORWARD);
        ejectionMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        ejectionMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ejectionMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize elevator and feeder servos
        Elevator = hw.get(Servo.class, "Elevator");
        Feeder = hw.get(Servo.class, "Feeder");
    }

    // ELEVATOR METHODS

    public void setElevatorPosition(double position) {
        Feeder.setPosition(position);
    }

    public void setElevatorUp() {
        setElevatorPosition(ELEVATOR_POS_UP);
    }

    public void setElevatorDown() {
        setElevatorPosition(ELEVATOR_POS_DOWN);
    }

    // FEEDER METHODS

    public void setFeederPosition(double position) {
        Feeder.setPosition(position);
    }

    public void setFeederUp() {
        setFeederPosition(FEEDER_POS_UP);
    }

    public void setFeederDown() {
        setFeederPosition(FEEDER_POS_DOWN);
    }

    // SHOTGUN METHODS

    public void setShotgunPower(double power) {
        ejectionMotorLeft.setPower(power);
        ejectionMotorRight.setPower(-power);
    }

    public void startShotgun() {
        setShotgunPower(SHOT_GUN_POWER_UP);
    }

    public void stopShotgun() {
        setShotgunPower(0);
    }

    public void update() {
        // For now, nothing non-blocking needed
        // Could later add state machine for shooting sequence
    }

    public void stop() {
        stopShotgun();
    }

}
