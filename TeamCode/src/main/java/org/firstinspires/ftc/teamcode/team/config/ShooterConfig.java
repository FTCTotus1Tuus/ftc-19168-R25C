package org.firstinspires.ftc.teamcode.team.config;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

/**
 * Shooter and flywheel tuning values.
 */
@Config
@Configurable
public class ShooterConfig {

    public static double SHOT_GUN_POWER_UP = 0.60;
    public static double SHOT_GUN_POWER_UP_FAR = 0.64;
    public static double SHOT_GUN_POWER_UP_RPM = 1100;
    public static double SHOT_GUN_POWER_UP_RPM_AUTO = 1075;
    public static double SHOT_GUN_POWER_UP_FAR_RPM_AUTO = 1350;
    public static double SHOT_GUN_POWER_UP_FAR_RPM_TELEOP = 1350;
    public static double SHOT_GUN_POWER_DOWN = 0.2;

    public static double SHOOTING_POWER_ODOMETRY_Y_THRESHOLD = 48.0;
    public static double SHOOT_POWER_SELECT_STICK_THRESHOLD = 0.05;

    public static double SHOT_GUN_PGAIN = 0.002;
    public static double SHOT_GUN_PGAIN2 = 0.0005;
    public static double SHOT_GUN_IGAIN = 0.00003;
    public static double SHOT_GUN_PDUTY_MIN = -0.5;
    public static double SHOT_GUN_PDUTY_MAX = 1.0;
    public static double SHOT_GUN_IDUTY_MIN = 0.0;
    public static double SHOT_GUN_IDUTY_MAX = 1.0;
    public static double SHOT_GUN_POWER_MIN = 0.0;
    public static double SHOT_GUN_POWER_MAX = 1.0;
    public static double SHOT_GUN_GAIN = 1.0;
    public static double SHOT_GUN_MIN_RPM = 0.0;
    public static double SHOT_GUN_MAX_RPM = 3000.0;

    private ShooterConfig() {
    }
}

