package org.firstinspires.ftc.teamcode.team.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * Shared path definitions for Red audience side autonomous options.
 * Audience side starts near the audience, shoots from distance (far shots).
 */
public class RedAudienceSidePaths {

    // Starting pose for Red alliance audience side
    public static double STARTING_POSE_X = 87;
    public static double STARTING_POSE_Y = 8.75;
    public static double STARTING_POSE_H_DEG = 90;

    public static PathChain buildShootingPosition1(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(STARTING_POSE_X, STARTING_POSE_Y),
                        new Pose(87.000, 18.000)
                )
        ).setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain buildIntakePos1(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(87.000, 18.000),
                        new Pose(100.000, 36.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();
    }

    public static PathChain buildIntakeBallSet1(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(100.000, 36.000),
                        new Pose(132.000, 36.000)
                )
        ).setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain buildShootingPosition2(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(132.000, 36.000),
                        new Pose(87.000, 18.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();
    }

    public static PathChain buildIntakePos2(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(87.000, 18.000),
                        new Pose(102.000, 58.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();
    }

    public static PathChain buildIntakeBallSet2(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(102.000, 58.000),
                        new Pose(129.000, 58.000)
                )
        ).setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain buildShootingPosition3(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(129.000, 58.000),
                        new Pose(70.000, 71.000),
                        new Pose(91.000, 45.000),
                        new Pose(87.000, 18.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public static PathChain buildParking(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(87.000, 18.000),
                        new Pose(87.000, 31.000)
                )
        ).setTangentHeadingInterpolation()
                .build();
    }
}

