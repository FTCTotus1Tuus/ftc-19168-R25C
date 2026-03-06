package org.firstinspires.ftc.teamcode.team.autosPedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import android.content.SharedPreferences;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootingFSM;

/**
 * Pedro Pathing auto using LinearOpMode via DarienOpModeFSM.
 */

@Autonomous(name = "Red Goal 6", group = "Pedro:Reds", preselectTeleOp = "TeleopFSM")
@Config
@Configurable
public class RedGoalSide1 extends DarienOpModeFSM {

    private TelemetryManager panelsTelemetry;   // Panels Telemetry instance
    // follower is inherited from DarienOpModeFSM
    private int pathState;                      // State machine state
    private Paths paths;                        // Paths
    private Timer pathTimer, opmodeTimer;
    private boolean shotgunRunning = false;     // Keep shotgun PID running continuously

    public static double STARTING_POSE_X = 111;
    public static double STARTING_POSE_Y = 134;
    public static double STARTING_POSE_H_DEG = 0;
    public static double PATH_POWER_STANDARD = 0.8;
    public static double PATH_POWER_SLOW = 0.4;
    public static double SHORT_PATH_TIMEOUT = 1.0;
    public static double STANDARD_PATH_TIMEOUT = 2.0;
    public static double LONG_PATH_TIMEOUT = 4.0;
    public static double SHOOT_TRIPLE_TIMEOUT = 4.0;
    public static double SHOOT_TRIPLE_TIME_MIN = 5.0;
    public static double SHOOT_TRIPLE_TIME_MAX = 7.0;
    public double targetGoalX = DarienOpModeFSM.GOAL_RED_X;
    public double targetGoalY = DarienOpModeFSM.GOAL_RED_Y;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- ROBOT + HARDWARE INIT (from DarienOpModeFSM) ---
        initControls(); // sets up TrayServo, Elevator, Feeder, motors, AprilTag, etc.

        // --- PEDRO + TIMERS INIT ---
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Starting pose
        follower.setStartingPose(new Pose(STARTING_POSE_X, STARTING_POSE_Y, Math.toRadians(STARTING_POSE_H_DEG)));

        // Build all the paths once
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        telemetry.addLine("RedGoalSidePedro: READY");
        panelsTelemetry.update(telemetry);

        turretFSM.center();

        // Save alliance color to shared preferences for TeleOp
        SharedPreferences prefs = AppUtil.getInstance().getActivity().getSharedPreferences("ftc_prefs", android.content.Context.MODE_PRIVATE);
        prefs.edit().putString("auto_alliance", "RED").apply();

        telemetry.addLine("Alliance Color: RED (Saved to Preferences)");


        // --- WAIT FOR START ---
        waitForStart();
        if (isStopRequested()) return;

        opmodeTimer.resetTimer();
        setPathState(0);

        targetGoalId = APRILTAG_ID_GOAL_RED;

        // --- MAIN AUTONOMOUS LOOP ---
        while (opModeIsActive() && !isStopRequested()) {

            // Pedro follower must be updated every loop
            follower.update();

            // Keep shotgun motor at target RPM every loop cycle (PID needs continuous updates)
            if (shotgunRunning) {
                shotgunFSM.toPowerUp(DarienOpModeFSM.SHOT_GUN_POWER_UP_RPM_AUTO);
            }

            double robotX = follower.getPose().getX();
            double robotY = follower.getPose().getY();
            double robotHeadingRadians = follower.getPose().getHeading();

            turretFSM.setPositionFromOdometry(targetGoalX, targetGoalY, robotX, robotY, robotHeadingRadians);

            // Save final odometry position to SharedPreferences for TeleOp
            prefs = AppUtil.getInstance().getActivity().getSharedPreferences("ftc_prefs", android.content.Context.MODE_PRIVATE);
            prefs.edit()
                    .putFloat("auto_final_x", (float) follower.getPose().getX())
                    .putFloat("auto_final_y", (float) follower.getPose().getY())
                    .putFloat("auto_final_heading", (float) follower.getPose().getHeading())
                    .apply();

            telemetry.addData("Saved Odometry", String.format("X=%.1f, Y=%.1f, H=%.1f°",
                                                              follower.getPose().getX(),
                                                              follower.getPose().getY(),
                                                              Math.toDegrees(follower.getPose().getHeading())));

            // Drive the state machine
            pathState = autonomousPathUpdate();

            // Panels/driver telemetry
            panelsTelemetry.addData("Path State", pathState);
            panelsTelemetry.addData("X", follower.getPose().getX());
            panelsTelemetry.addData("Y", follower.getPose().getY());
            panelsTelemetry.addData("Heading", follower.getPose().getHeading());
            panelsTelemetry.addData("Alliance Color", "RED");
            telemetry.addData("Alliance Color Saved", "RED");
            displayRpmTelemetry();
            panelsTelemetry.update(telemetry);
        }
    }


    /**
     * Inner class defining all the Pedro paths.
     */
    public static class Paths {
        public PathChain ShootingPosition1;
        public PathChain IntakePos1;
        public PathChain IntakeBallSet1;
        public PathChain ShootingPosition2;
        public PathChain Parking;

        public Paths(Follower follower) {
            ShootingPosition1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(STARTING_POSE_X, STARTING_POSE_Y),

                                    new Pose(87.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(STARTING_POSE_H_DEG), Math.toRadians(0))

                    .build();

            IntakePos1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.000, 84.000),

                                    new Pose(100.000, 84.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            IntakeBallSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(100.000, 84.000),

                                    new Pose(125.000, 84.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ShootingPosition2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.000, 84.000),

                                    new Pose(87.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Parking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.000, 84.000),

                                    new Pose(87.000, 120.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();
        }
    }


    /**
     * State machine for autonomous sequence.
     * Returns current pathState for logging.
     */
    public int autonomousPathUpdate() {

        // Helpful debug info every loop
        telemetry.addData("PathState", pathState);
        telemetry.addData("FollowerBusy", follower.isBusy());
        telemetry.addData("PathTimer", pathTimer.getElapsedTimeSeconds());

        switch (pathState) {
            case 0:
                // move to shooting position 1, start shotgun spinning immediately
                follower.setMaxPower(PATH_POWER_STANDARD);
                intakeFSM.setModeFull();
                shotgunRunning = true;  // start shotgun — PID runs every loop via main loop
                follower.followPath(paths.ShootingPosition1);
                setPathState(pathState + 1);
                break;

            case 1:
                // when at shootingPosition 1, start shooting
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {
                    shootingFSM.start(getRuntime(), ShootingFSM.PowerLevel.CLOSE);
                    setPathState(pathState + 1);
                }
                break;

            case 2:
                // when shooting is done, move to intakePos1
                shootingFSM.update(getRuntime(), telemetry);
                telemetry.addLine("Case " + pathState + ": Start IntakeBallSet1");
                if (SHOOT_TRIPLE_TIME_MIN < pathTimer.getElapsedTimeSeconds() && pathTimer.getElapsedTimeSeconds() < SHOOT_TRIPLE_TIME_MAX) {
                    shootingFSM.reset();
                    gateFSM.close();  // close gate to prevent illegal shooting while moving
                    // shotgun stays spinning — no toOff() here
                    follower.followPath(paths.IntakePos1);
                    setPathState(pathState + 1);
                }
                break;

            case 3:
                //when in intakepos1, intake ball set 1
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {
                    follower.setMaxPower(PATH_POWER_SLOW);
                    intakeFSM.startIntaking();
                    follower.followPath(paths.IntakeBallSet1);
                    setPathState(pathState + 1);
                }
                break;

            case 4:
                //after intaking, move to park
                intakeFSM.updateIntaking(getRuntime(), true, telemetry);
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {
                    follower.setMaxPower(PATH_POWER_STANDARD);
                    follower.followPath(paths.Parking, true);
                    setPathState(pathState + 1);
                }
                break;

            case 5:
                //after parking, start shooting
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > LONG_PATH_TIMEOUT) {
                    shootingFSM.start(getRuntime(), ShootingFSM.PowerLevel.CLOSE);
                    setPathState(pathState + 1);
                }
                break;

            case 6:
                //after done shooting, send odometry values to teleop
                shootingFSM.update(getRuntime(), telemetry);
                if (SHOOT_TRIPLE_TIME_MIN < pathTimer.getElapsedTimeSeconds() && pathTimer.getElapsedTimeSeconds() < SHOOT_TRIPLE_TIME_MAX) {
                    shootingFSM.reset();
                    shotgunRunning = false;  // stop continuous PID loop
                    shotgunFSM.toOff();
                    intakeFSM.off();
                    telemetry.addLine("Case " + pathState + ": Done, setting state -1");
                    setPathState(-1); // done
                }
                break;

            default:
                // -1 or any undefined state: do nothing, stay idle
                telemetry.addLine("Idle state (pathState = " + pathState + ")");
                break;
        }

        return pathState;
    }

    /**
     * Sets the path state and resets its timer.
     */
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}