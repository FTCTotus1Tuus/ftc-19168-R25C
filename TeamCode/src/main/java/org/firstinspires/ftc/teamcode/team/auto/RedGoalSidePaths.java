package org.firstinspires.ftc.teamcode.team.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * Shared path definitions for Red side autonomous options.
 * Each path builder is kept as a static method for flexibility;
 * concrete autos can call only the paths they need.
 */
public class RedGoalSidePaths {

    // Starting pose constants (tunable via @Config)
    public static double STARTING_POSE_X = 111;
    public static double STARTING_POSE_Y = 134;
    public static double STARTING_POSE_H_DEG = 0;

    // Common paths for RedGoalSide1 and RedGoalSide2
    public static PathChain buildShootingPosition1(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(STARTING_POSE_X, STARTING_POSE_Y),
                        new Pose(87.000, 84.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(STARTING_POSE_H_DEG), Math.toRadians(0))
                .build();
    }

    public static PathChain buildIntakePos1(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(87.000, 84.000),
                        new Pose(100.000, 84.000)
                )
        ).setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain buildIntakeBallSet1(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(100.000, 84.000),
                        new Pose(125.000, 84.000)
                )
        ).setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain buildShootingPosition2(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(125.000, 84.000),
                        new Pose(87.000, 84.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public static PathChain buildParking1(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(87.000, 84.000),
                        new Pose(87.000, 120.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public static PathChain buildIntakePos2(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(87.000, 84.000),
                        new Pose(100.000, 60.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public static PathChain buildIntakeBallSet2(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(100.000, 60.000),
                        new Pose(129.500, 60.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public static PathChain buildParking2Curve(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(129.500, 60.000),
                        new Pose(78.110, 56.335),
                        new Pose(87.808, 88.344),
                        new Pose(87.000, 120.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
}
