package org.firstinspires.ftc.teamcode.team.config;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

/**
 * Turret aiming geometry and servo tuning values.
 */
@Config
@Configurable
public class TurretConfig {

    public static double GOAL_RED_X = 144.0;
    public static double GOAL_RED_Y = 144.0;
    public static double GOAL_BLUE_X = 0.0;
    public static double GOAL_BLUE_Y = 144.0;

    public static double TURRET_GEAR_RATIO = 5.8;
    public static double TURRET_ROTATION_INCREMENT = 0.001;
    public static double TURRET_ROTATION_INCREMENT_FAST = 0.003;
    public static double TURRET_POSITION_CENTER = 0.5;
    public static double TURRET_OFFSET_DEG_RED = 1.0;
    public static double TURRET_OFFSET_DEG_BLUE = -3.0;
    public static double TURRET_PIVOT_OFFSET_INCHES = -1.5;
    public static double TURRET_PIVOT_OFFSET_LATERAL_INCHES = 0.0;
    public static double TURRET_MAX_DEG_LEFT = 150.0;
    public static double TURRET_MAX_DEG_RIGHT = 180.0;

    private TurretConfig() {
    }
}

