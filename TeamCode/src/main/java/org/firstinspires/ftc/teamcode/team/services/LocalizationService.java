package org.firstinspires.ftc.teamcode.team.services;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Owns Pinpoint pose seed/reset operations and keeps follower pose synchronized.
 */
public class LocalizationService {

    public static class SeedResult {
        public final boolean loadedFromAuto;
        public final double x;
        public final double y;
        public final double headingRad;

        public SeedResult(boolean loadedFromAuto, double x, double y, double headingRad) {
            this.loadedFromAuto = loadedFromAuto;
            this.x = x;
            this.y = y;
            this.headingRad = headingRad;
        }
    }

    private final GoBildaPinpointDriver odo;
    private final Follower follower;

    public LocalizationService(HardwareMap hardwareMap, Follower follower) {
        this.odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        this.follower = follower;
    }

    public SeedResult seedTeleOpPose(
            String autoAlliance,
            PreferencesService preferencesService,
            double humanPlayerRedX,
            double humanPlayerRedY,
            double humanPlayerBlueX,
            double humanPlayerBlueY,
            double robotCenterOffsetX,
            double robotCenterOffsetY
    ) {
        if (preferencesService.hasAutoFinalPose()) {
            double autoX = preferencesService.getAutoFinalX(0f);
            double autoY = preferencesService.getAutoFinalY(0f);
            double autoHeadingRad = preferencesService.getAutoFinalHeading(0f);
            applyPoseInchesRadians(autoX, autoY, autoHeadingRad);
            return new SeedResult(true, autoX, autoY, autoHeadingRad);
        }

        double resetX = 0;
        double resetY = 0;
        double resetHeadingRad = 0;

        if ("RED".equals(autoAlliance)) {
            resetX = humanPlayerRedX + robotCenterOffsetX;
            resetY = humanPlayerRedY + robotCenterOffsetY;
            resetHeadingRad = Math.PI;
        } else if ("BLUE".equals(autoAlliance)) {
            resetX = humanPlayerBlueX - robotCenterOffsetX;
            resetY = humanPlayerBlueY + robotCenterOffsetY;
            resetHeadingRad = 0;
        }

        applyPoseInchesRadians(resetX, resetY, resetHeadingRad);
        return new SeedResult(false, resetX, resetY, resetHeadingRad);
    }

    public SeedResult resetToHumanPlayerPosition(
            String autoAlliance,
            double humanPlayerRedX,
            double humanPlayerRedY,
            double humanPlayerBlueX,
            double humanPlayerBlueY,
            double robotCenterOffsetX,
            double robotCenterOffsetY
    ) {
        double resetX = 0;
        double resetY = 0;
        double resetHeadingRad = 0;

        if ("RED".equals(autoAlliance)) {
            resetX = humanPlayerRedX + robotCenterOffsetX;
            resetY = humanPlayerRedY + robotCenterOffsetY;
            resetHeadingRad = Math.PI;
        } else if ("BLUE".equals(autoAlliance)) {
            resetX = humanPlayerBlueX - robotCenterOffsetX;
            resetY = humanPlayerBlueY + robotCenterOffsetY;
            resetHeadingRad = 0;
        }

        applyPoseInchesRadians(resetX, resetY, resetHeadingRad);
        return new SeedResult(false, resetX, resetY, resetHeadingRad);
    }

    private void applyPoseInchesRadians(double x, double y, double headingRad) {
        odo.setPosition(new Pose2D(
                DistanceUnit.INCH,
                x,
                y,
                AngleUnit.RADIANS,
                headingRad
        ));
        follower.setPose(new Pose(x, y, headingRad));
    }
}

