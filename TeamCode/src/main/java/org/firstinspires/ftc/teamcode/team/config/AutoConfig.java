package org.firstinspires.ftc.teamcode.team.config;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

/**
 * Autonomous and field-position tuning values.
 */
@Config
@Configurable
public class AutoConfig {

    public static double HUMAN_PLAYER_RED_X = 0.0;
    public static double HUMAN_PLAYER_RED_Y = 0.0;
    public static double HUMAN_PLAYER_BLUE_X = 144.0;
    public static double HUMAN_PLAYER_BLUE_Y = 0.0;

    public static double PARK_RED_X = 39.0;
    public static double PARK_RED_Y = 33.0;
    public static double PARK_RED_H_DEG = 180.0;
    public static double PARK_BLUE_X = 105.0;
    public static double PARK_BLUE_Y = 33.0;
    public static double PARK_BLUE_H_DEG = 0.0;

    public static double AUTO_PARK_TIMEOUT = 10.0;
    public static double AUTO_PARK_POWER = 0.7;
    public static double AUTO_PARK_STICK_DEADZONE = 0.1;

    private AutoConfig() {
    }
}

