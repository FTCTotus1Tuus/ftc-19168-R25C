package org.firstinspires.ftc.teamcode.team.core;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Maps raw gamepad state into immutable TeleOp driver intent snapshots.
 */
public class TeleOpInputMapper {

    public DriverOneBindings mapDriverOne(Gamepad gamepad) {
        return new DriverOneBindings(
                gamepad.left_stick_y,
                gamepad.left_stick_x,
                gamepad.right_stick_x,
                gamepad.y || gamepad.right_bumper,
                gamepad.a,
                gamepad.x,
                gamepad.bWasPressed(),
                gamepad.dpadUpWasPressed()
        );
    }

    public DriverTwoBindings mapDriverTwo(Gamepad gamepad) {
        return new DriverTwoBindings(
                gamepad.left_bumper,
                gamepad.rightBumperWasPressed(),
                gamepad.rightBumperWasReleased(),
                gamepad.right_stick_y,
                gamepad.b,
                gamepad.x,
                gamepad.dpadUpWasPressed(),
                gamepad.dpadDownWasPressed(),
                gamepad.left_stick_x,
                gamepad.left_trigger,
                gamepad.left_stick_button,
                gamepad.rightStickButtonWasPressed(),
                gamepad.a
        );
    }
}

