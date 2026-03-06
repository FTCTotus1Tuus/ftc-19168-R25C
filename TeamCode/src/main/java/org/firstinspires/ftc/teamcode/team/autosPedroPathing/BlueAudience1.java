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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import android.content.SharedPreferences;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootingFSM;

/**
 * Pedro Pathing auto using LinearOpMode via DarienOpModeFSM.
 */

@Autonomous(name = "Blue Audience 6", group = "Pedro:Blues", preselectTeleOp = "TeleopFSM")
@Configurable
@Config
public class BlueAudience1 extends DarienOpModeFSM {
    private TelemetryManager panelsTelemetry;   // Panels Telemetry instance
    // follower is inherited from DarienOpModeFSM
    private int pathState;                      // State machine state
    private Paths paths;                        // Paths
    private Timer pathTimer, opmodeTimer;
    private boolean shotgunRunning = false;     // Keep shotgun PID running continuously

    public static double PATH_POWER_STANDARD = 0.8;
    public static double PATH_POWER_SLOW = 0.4;
    public static double SHORT_PATH_TIMEOUT = 1.0;
    public static double STANDARD_PATH_TIMEOUT = 2.0;
    public static double LONG_PATH_TIMEOUT = 4.0;
    public static double SHOOT_TRIPLE_TIMEOUT = 4.0;
    public static double TURRET_ROTATE_DELAY = 4; //FOR AUDIENCE SIDE
    public static double SHOOT_TRIPLE_TIME_MIN = 5.0;
    public static double SHOOT_TRIPLE_TIME_MAX = 7.0;
    public double targetGoalX = DarienOpModeFSM.GOAL_BLUE_X;
    public double targetGoalY = DarienOpModeFSM.GOAL_BLUE_Y;

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
        follower.setStartingPose(new Pose(57, 8.75, Math.toRadians(90)));

        // Build all the paths once
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        telemetry.addLine("BlueAudienceSidePedro: READY");
        panelsTelemetry.update(telemetry);

        turretFSM.center();

        // Save alliance color to shared preferences for TeleOp
        SharedPreferences prefs = AppUtil.getInstance().getActivity().getSharedPreferences("ftc_prefs", android.content.Context.MODE_PRIVATE);
        prefs.edit().putString("auto_alliance", "BLUE").apply();

        telemetry.addLine("Alliance Color: BLUE (Saved to Preferences)");


        // --- WAIT FOR START ---
        waitForStart();
        if (isStopRequested()) return;

        opmodeTimer.resetTimer();
        setPathState(0);

        targetGoalId = APRILTAG_ID_GOAL_BLUE;

        // --- MAIN AUTONOMOUS LOOP ---
        while (opModeIsActive() && !isStopRequested()) {

            // Pedro follower must be updated every loop
            follower.update();

            // Keep shotgun motor at target RPM every loop cycle (PID needs continuous updates)
            if (shotgunRunning) {
                shotgunFSM.toPowerUp(DarienOpModeFSM.SHOT_GUN_POWER_UP_FAR_RPM_AUTO);
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
            panelsTelemetry.addData("Alliance Color", "BLUE");
            telemetry.addData("Alliance Color Saved", "BLUE");
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
        public PathChain IntakePos2;
        public PathChain IntakeBallSet2;
        public PathChain ShootingPosition3;
        public PathChain Parking;

        public Paths(Follower follower) {
            ShootingPosition1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.000, 8.750),

                                    new Pose(57.000, 18.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            IntakePos1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.000, 18.000),

                                    new Pose(42.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            IntakeBallSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.000, 36.000),

                                    new Pose(12.000, 36.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ShootingPosition2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.000, 36.000),

                                    new Pose(57.000, 18.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                    .build();

            IntakePos2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.000, 18.000),

                                    new Pose(42.000, 58.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            IntakeBallSet2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.000, 58.000),

                                    new Pose(14.500, 58.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ShootingPosition3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(14.500, 58.000),
                                    new Pose(56.000, 70.000),
                                    new Pose(56.000, 55.000),
                                    new Pose(57.000, 18.000)
                    )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Parking = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(57.000, 18.000),

                                    new Pose(57.000, 31.000)
                    )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
    }



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
                    shootingFSM.start(getRuntime(), ShootingFSM.PowerLevel.FAR);
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
                //after intaking, move to shooting pos 2
                intakeFSM.updateIntaking(getRuntime(), true, telemetry);
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {
                    follower.setMaxPower(PATH_POWER_STANDARD); //speed up again
                    follower.followPath(paths.ShootingPosition2, true);
                    setPathState(pathState + 1);
                }
                break;

            case 5:
                //when at shooting 2 then start shooting
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > TURRET_ROTATE_DELAY) {
                    shootingFSM.start(getRuntime(), ShootingFSM.PowerLevel.FAR);
                    setPathState(pathState + 1);
                }
                break;

            case 6:
                //after done shooting, send odometry values to teleop
                shootingFSM.update(getRuntime(), telemetry);
                if (SHOOT_TRIPLE_TIME_MIN < pathTimer.getElapsedTimeSeconds() && pathTimer.getElapsedTimeSeconds() < SHOOT_TRIPLE_TIME_MAX) {
                    follower.followPath(paths.Parking, true);
                    shootingFSM.reset();
                    shotgunRunning = false;  // stop continuous PID loop
                    shotgunFSM.toOff();
                    intakeFSM.off();
                    setPathState(pathState + 1);
                }
                break;

            case 7:
                // finish the move to parking
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {
                    setPathState(-1);
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