package org.firstinspires.ftc.teamcode.darienbot.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.darienbot.BaseOpMode;
import org.firstinspires.ftc.teamcode.darienbot.commands.*;
import org.firstinspires.ftc.teamcode.darienbot.subsystems.TraySubsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Autonomous(name = "First Auto Queue", group = "DarienBot")
public class FirstAutoQueue extends BaseOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // BaseOpMode handles robot initialization
        super.runOpMode();

        // -----------------------------
        // STARTING POSE
        // -----------------------------
        robot.drive.setPose(new Pose(123.643, 122.308, Math.toRadians(135)));

        // -----------------------------
        // BUILD EXPORTED PATHS
        // -----------------------------
        Paths paths = new Paths(robot.drive.getFollower());


        // ---------------------------
        // WAIT FOR START
        // ---------------------------
        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;


        // ---------------------------
        // SCHEDULE COMMAND SEQUENCE
        // ---------------------------

        // 1. Follow strafeLeft path chain
        scheduler.schedule(new FollowPathChainCommand(robot.drive, paths.strafeLeft));

        // 2. Move tray into scoring position
        scheduler.schedule(new TrayCommand(
                robot.tray,
                TraySubsystem.TRAY_POS_2_SCORE,
                TraySubsystem.Owner.SCORING
        ));

        // 3. Follow rotateToRedGoal path chain
        scheduler.schedule(new FollowPathChainCommand(robot.drive, paths.rotateToRedGoal));

        // 4. Shoot one artifact (non-blocking timed command)
        scheduler.schedule(new ShootArtifactCommand(robot.scoring));

        // 5. Return tray to intake position
        scheduler.schedule(new TrayCommand(
                robot.tray,
                TraySubsystem.TRAY_POS_2_INTAKE,
                TraySubsystem.Owner.INTAKE
        ));

        // ---------------------------
        // AUTO LOOP
        // ---------------------------
        while (opModeIsActive() && !scheduler.isFinished()) {
            scheduler.update();   // run active command
            robot.update();       // update subsystems (drive, tray, etc.)

            // Optional debug
            // --- Telemetry: Active command name ---
            Command active = scheduler.getCurrentCommand();
            if (active != null) {
                telemetry.addData("Active Command", active.getClass().getSimpleName());
            } else {
                telemetry.addData("Active Command", "None");
            }
            telemetry.addData("TrayPos", robot.tray.getPosition());
            
            Pose pose = robot.drive.getPose();
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));
            telemetry.update();
        }
        // After auto ends, stop motors safely
        robot.stop();
    }

    public static class Paths {

        public PathChain strafeLeft;
        public PathChain rotateToRedGoal;

        public Paths(Follower follower) {
            // Matches your exporter code:

            strafeLeft = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(123.643, 122.308),
                                    new Pose(100.950, 100.116)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(135),
                            Math.toRadians(120)
                    )
                    .build();

            rotateToRedGoal = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(100.950, 100.116),
                                    new Pose(100.950, 100.116)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(135),
                            Math.toRadians(45)
                    )
                    .build();
        }
    }
}
