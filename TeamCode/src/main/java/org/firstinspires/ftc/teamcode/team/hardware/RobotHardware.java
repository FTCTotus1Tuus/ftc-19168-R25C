package org.firstinspires.ftc.teamcode.team.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Central hardware mapping for robot devices used by shared FSM infrastructure.
 */
public class RobotHardware {

    public DcMotorEx ejectionMotor;

    public void initialize(HardwareMap hardwareMap) {
        ejectionMotor = hardwareMap.get(DcMotorEx.class, "ejectionMotor");
        ejectionMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        ejectionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ejectionMotor.setDirection(DcMotorEx.Direction.REVERSE); // Reverse because it is geared.
    }
}

