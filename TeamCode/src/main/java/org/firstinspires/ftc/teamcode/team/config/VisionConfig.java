package org.firstinspires.ftc.teamcode.team.config;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

/**
 * Vision + AprilTag tuning values.
 */
@Config
@Configurable
public class VisionConfig {

    public static int APRILTAG_ID_GOAL_BLUE = 20;
    public static int APRILTAG_ID_GOAL_RED = 24;

    public static int APRILTAG_EXPOSURE_MS = 6;
    public static int APRILTAG_GAIN = 255;

    public static double TIMEOUT_APRILTAG_DETECTION = 0.75;
    public static double CAMERA_FALLBACK_TIMEOUT_MS = 500.0;

    private VisionConfig() {
    }
}

