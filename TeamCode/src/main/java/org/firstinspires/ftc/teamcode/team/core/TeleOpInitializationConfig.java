package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.teamcode.team.config.AutoConfig;
import org.firstinspires.ftc.teamcode.team.config.DriveConfig;

/**
 * Startup constants used for TeleOp auto-state restoration and pose seeding.
 */
public class TeleOpInitializationConfig {
    public final String unknownAllianceDefault;
    public final double humanPlayerRedX;
    public final double humanPlayerRedY;
    public final double humanPlayerBlueX;
    public final double humanPlayerBlueY;
    public final double robotCenterOffsetX;
    public final double robotCenterOffsetY;

    public TeleOpInitializationConfig(
            String unknownAllianceDefault,
            double humanPlayerRedX,
            double humanPlayerRedY,
            double humanPlayerBlueX,
            double humanPlayerBlueY,
            double robotCenterOffsetX,
            double robotCenterOffsetY
    ) {
        this.unknownAllianceDefault = unknownAllianceDefault;
        this.humanPlayerRedX = humanPlayerRedX;
        this.humanPlayerRedY = humanPlayerRedY;
        this.humanPlayerBlueX = humanPlayerBlueX;
        this.humanPlayerBlueY = humanPlayerBlueY;
        this.robotCenterOffsetX = robotCenterOffsetX;
        this.robotCenterOffsetY = robotCenterOffsetY;
    }

    public static TeleOpInitializationConfig createDefault() {
        return new TeleOpInitializationConfig(
                "UNKNOWN",
                AutoConfig.HUMAN_PLAYER_RED_X,
                AutoConfig.HUMAN_PLAYER_RED_Y,
                AutoConfig.HUMAN_PLAYER_BLUE_X,
                AutoConfig.HUMAN_PLAYER_BLUE_Y,
                DriveConfig.ROBOT_CENTER_OFFSET_X,
                DriveConfig.ROBOT_CENTER_OFFSET_Y
        );
    }
}

