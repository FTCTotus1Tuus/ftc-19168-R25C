package org.firstinspires.ftc.teamcode.team.core;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/**
 * Coordinates TeleOp odometry reset behavior.
 */
public class OdometryResetCoordinator {

    public void tryResetToHumanPlayerPosition(
            boolean resetRequested,
            String autoAlliance,
            double humanPlayerRedX,
            double humanPlayerRedY,
            double humanPlayerBlueX,
            double humanPlayerBlueY,
            double robotCenterOffsetX,
            double robotCenterOffsetY,
            GoBildaPinpointDriver odo,
            Follower follower,
            Telemetry telemetry
    ) {
        if (!resetRequested) {
            return;
        }

        double resetX = 0;
        double resetY = 0;
        double resetHeadingDeg = 0;

        if ("RED".equals(autoAlliance)) {
            resetX = humanPlayerRedX + robotCenterOffsetX;
            resetY = humanPlayerRedY + robotCenterOffsetY;
            resetHeadingDeg = 180;
            telemetry.addLine("ODOMETRY RESET: Red Human Player Position (0, 0)");
        } else if ("BLUE".equals(autoAlliance)) {
            resetX = humanPlayerBlueX - robotCenterOffsetX;
            resetY = humanPlayerBlueY + robotCenterOffsetY;
            resetHeadingDeg = 0;
            telemetry.addLine("ODOMETRY RESET: Blue Human Player Position (144, 0)");
        }

        odo.setPosition(new Pose2D(
                DistanceUnit.INCH,
                resetX,
                resetY,
                AngleUnit.DEGREES,
                resetHeadingDeg
        ));

        follower.setPose(new Pose(resetX, resetY, Math.toRadians(resetHeadingDeg)));
        telemetry.addData("New Odometry Position", String.format(Locale.US, "(%.1f, %.1f, %.1f)", resetX, resetY, resetHeadingDeg));
    }
}


