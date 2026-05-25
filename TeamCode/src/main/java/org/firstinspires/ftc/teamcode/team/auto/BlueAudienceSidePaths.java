package org.firstinspires.ftc.teamcode.team.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * Shared path definitions for Blue audience side autonomous options.
 * Mirrors Red audience (x-coordinates flipped).
 */
public class BlueAudienceSidePaths {

    // Starting pose for Blue alliance audience side (mirrored from Red)
    public static double STARTING_POSE_X = 57;
    public static double STARTING_POSE_Y = 8.75;
    public static double STARTING_POSE_H_DEG = 90;

    public static PathChain buildShootingPosition1(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(STARTING_POSE_X, STARTING_POSE_Y),
                        new Pose(57.000, 18.000)
                )
        ).setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain buildIntakePos1(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(57.000, 18.000),
                        new Pose(44.000, 36.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
    }

    public static PathChain buildIntakeBallSet1(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(44.000, 36.000),
                        new Pose(12.000, 36.000)
                )
        ).setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain buildShootingPosition2(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(12.000, 36.000),
                        new Pose(57.000, 18.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();
    }

    public static PathChain buildIntakePos2(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(57.000, 18.000),
                        new Pose(42.000, 58.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
    }

    public static PathChain buildIntakeBallSet2(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(42.000, 58.000),
                        new Pose(15.000, 58.000)
                )
        ).setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain buildShootingPosition3(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(15.000, 58.000),
                        new Pose(74.000, 71.000),
                        new Pose(53.000, 45.000),
                        new Pose(57.000, 18.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain buildParking(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(57.000, 18.000),
                        new Pose(57.000, 31.000)
                )
        ).setTangentHeadingInterpolation()
                .build();
    }
}

