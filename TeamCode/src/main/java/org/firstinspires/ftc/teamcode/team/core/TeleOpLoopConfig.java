package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.teamcode.team.config.AutoConfig;
import org.firstinspires.ftc.teamcode.team.config.DriveConfig;
import org.firstinspires.ftc.teamcode.team.config.ShooterConfig;

/**
 * Bundles the TeleOp tuning constants used by the loop context and dependencies.
 */
public class TeleOpLoopConfig {
    public final double autoParkStickDeadzone;
    public final double autoParkTimeout;
    public final double driveDeadzone;
    public final double inputExponent;
    public final double speedScale;
    public final double speedScaleTurn;
    public final double rotationScale;
    public final double parkRedX;
    public final double parkRedY;
    public final double parkRedHeadingDeg;
    public final double parkBlueX;
    public final double parkBlueY;
    public final double parkBlueHeadingDeg;
    public final double autoParkPower;
    public final double humanPlayerRedX;
    public final double humanPlayerRedY;
    public final double humanPlayerBlueX;
    public final double humanPlayerBlueY;
    public final double robotCenterOffsetX;
    public final double robotCenterOffsetY;
    public final double shootingPowerOdometryYThreshold;
    public final double shootPowerSelectStickThreshold;
    public final double closeRpm;
    public final double farRpm;

    public TeleOpLoopConfig(
            double autoParkStickDeadzone,
            double autoParkTimeout,
            double driveDeadzone,
            double inputExponent,
            double speedScale,
            double speedScaleTurn,
            double rotationScale,
            double parkRedX,
            double parkRedY,
            double parkRedHeadingDeg,
            double parkBlueX,
            double parkBlueY,
            double parkBlueHeadingDeg,
            double autoParkPower,
            double humanPlayerRedX,
            double humanPlayerRedY,
            double humanPlayerBlueX,
            double humanPlayerBlueY,
            double robotCenterOffsetX,
            double robotCenterOffsetY,
            double shootingPowerOdometryYThreshold,
            double shootPowerSelectStickThreshold,
            double closeRpm,
            double farRpm
    ) {
        this.autoParkStickDeadzone = autoParkStickDeadzone;
        this.autoParkTimeout = autoParkTimeout;
        this.driveDeadzone = driveDeadzone;
        this.inputExponent = inputExponent;
        this.speedScale = speedScale;
        this.speedScaleTurn = speedScaleTurn;
        this.rotationScale = rotationScale;
        this.parkRedX = parkRedX;
        this.parkRedY = parkRedY;
        this.parkRedHeadingDeg = parkRedHeadingDeg;
        this.parkBlueX = parkBlueX;
        this.parkBlueY = parkBlueY;
        this.parkBlueHeadingDeg = parkBlueHeadingDeg;
        this.autoParkPower = autoParkPower;
        this.humanPlayerRedX = humanPlayerRedX;
        this.humanPlayerRedY = humanPlayerRedY;
        this.humanPlayerBlueX = humanPlayerBlueX;
        this.humanPlayerBlueY = humanPlayerBlueY;
        this.robotCenterOffsetX = robotCenterOffsetX;
        this.robotCenterOffsetY = robotCenterOffsetY;
        this.shootingPowerOdometryYThreshold = shootingPowerOdometryYThreshold;
        this.shootPowerSelectStickThreshold = shootPowerSelectStickThreshold;
        this.closeRpm = closeRpm;
        this.farRpm = farRpm;
    }

    public static TeleOpLoopConfig createDefault() {
        return new TeleOpLoopConfig(
                AutoConfig.AUTO_PARK_STICK_DEADZONE,
                AutoConfig.AUTO_PARK_TIMEOUT,
                DriveConfig.DRIVE_DEADZONE,
                DriveConfig.INPUT_EXPONENT,
                DriveConfig.SPEED_SCALE,
                DriveConfig.SPEED_SCALE_TURN,
                DriveConfig.ROTATION_SCALE,
                AutoConfig.PARK_RED_X,
                AutoConfig.PARK_RED_Y,
                AutoConfig.PARK_RED_H_DEG,
                AutoConfig.PARK_BLUE_X,
                AutoConfig.PARK_BLUE_Y,
                AutoConfig.PARK_BLUE_H_DEG,
                AutoConfig.AUTO_PARK_POWER,
                AutoConfig.HUMAN_PLAYER_RED_X,
                AutoConfig.HUMAN_PLAYER_RED_Y,
                AutoConfig.HUMAN_PLAYER_BLUE_X,
                AutoConfig.HUMAN_PLAYER_BLUE_Y,
                DriveConfig.ROBOT_CENTER_OFFSET_X,
                DriveConfig.ROBOT_CENTER_OFFSET_Y,
                ShooterConfig.SHOOTING_POWER_ODOMETRY_Y_THRESHOLD,
                ShooterConfig.SHOOT_POWER_SELECT_STICK_THRESHOLD,
                ShooterConfig.SHOT_GUN_POWER_UP_RPM,
                ShooterConfig.SHOT_GUN_POWER_UP_FAR_RPM_TELEOP
        );
    }
}

