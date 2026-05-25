package org.firstinspires.ftc.teamcode.team.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * Shared path definitions for Blue goal side autonomous options.
 * Mirrors Red goal side (x-coordinates flipped across field center).
 */
public class BlueGoalSidePaths {

    // Starting pose constants for Blue alliance (mirrored from Red)
    public static double STARTING_POSE_X = 33;
    public static double STARTING_POSE_Y = 134;
    public static double STARTING_POSE_H_DEG = 180;

    // Common paths for BlueGoalSide1 and BlueGoalSide2
    public static PathChain buildShootingPosition1(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(STARTING_POSE_X, STARTING_POSE_Y),
                        new Pose(57.000, 84.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(STARTING_POSE_H_DEG), Math.toRadians(180))
                .build();
    }

    public static PathChain buildIntakePos1(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(57.000, 84.000),
                        new Pose(44.000, 84.000)
                )
        ).setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain buildIntakeBallSet1(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(44.000, 84.000),
                        new Pose(19.000, 84.000)
                )
        ).setTangentHeadingInterpolation()
                .build();
    }

    public static PathChain buildShootingPosition2(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(19.000, 84.000),
                        new Pose(57.000, 84.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain buildParking1(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(57.000, 84.000),
                        new Pose(57.000, 120.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain buildIntakePos2(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(57.000, 84.000),
                        new Pose(44.000, 60.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain buildIntakeBallSet2(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(44.000, 60.000),
                        new Pose(14.500, 60.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    public static PathChain buildParking2Curve(Follower follower) {
        return follower.pathBuilder().addPath(
                new BezierLine(
                        new Pose(14.500, 60.000),
                        new Pose(57.000, 120.000)
                )
        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
}

