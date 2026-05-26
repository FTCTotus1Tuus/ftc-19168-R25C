package org.firstinspires.ftc.teamcode.team.config;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

/**
 * Drive + localization tuning values shared by TeleOp and autonomous.
 */
@Config
@Configurable
public class DriveConfig {

    // Encoder constants for goBILDA 5203 in 4x mode.
    public static double TICKS_PER_ROTATION = 28 * 4;

    // Human-player reset origin offset from robot center.
    public static double ROBOT_CENTER_OFFSET_X = 8.5;
    public static double ROBOT_CENTER_OFFSET_Y = 8.25;

    // TeleOp drive shaping.
    public static double ROTATION_SCALE = 0.5;
    public static double SPEED_SCALE = 1.0;
    public static double SPEED_SCALE_TURN = 0.8;
    public static double INPUT_EXPONENT = 3.0;
    public static double DRIVE_DEADZONE = 0.1;

    private DriveConfig() {
    }
}

