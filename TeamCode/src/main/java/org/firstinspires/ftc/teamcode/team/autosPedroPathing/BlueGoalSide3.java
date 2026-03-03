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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootingFSM;

/**
 * Pedro Pathing auto using LinearOpMode via DarienOpModeFSM.
 */

@Autonomous(name = "Blue Goal 12", group = "Pedro:Blues", preselectTeleOp = "TeleopFSM")
@Config
@Configurable
public class BlueGoalSide3 extends DarienOpModeFSM {

    private TelemetryManager panelsTelemetry;   // Panels Telemetry instance
    public Follower follower;                   // Pedro Pathing follower instance
    private int pathState;                      // State machine state
    private Paths paths;                        // Paths
    private Timer pathTimer, opmodeTimer;

    public static double PATH_POWER_STANDARD = 0.5;
    public static double PATH_POWER_SLOW = 0.2;


    public static double INTAKE_RUBBER_BANDS_DELAY = 0.2;
    public static double BALL_INTAKE_DELAY = 0;
    public static double SHOTGUN_SPINUP_DELAY = 0;
    public static double STANDARD_PATH_TIMEOUT = 2.0;
    public static double SHOOT_TRIPLE_TIMEOUT = 7.0;
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

        follower = Constants.createFollower(hardwareMap);
        // Starting pose – same as your OpMode version
        follower.setStartingPose(new Pose(57, 135, Math.toRadians(180)));

        // Build all the paths once
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        telemetry.addLine("BlueGoalSidePedro: READY");
        telemetry.update();

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

            double robotX = follower.getPose().getX();
            double robotY = follower.getPose().getY();
            double robotHeadingRadians = follower.getPose().getHeading();

            turretFSM.setPositionFromOdometry(targetGoalX, targetGoalY, robotX, robotY, robotHeadingRadians);

            // Drive the state machine
            pathState = autonomousPathUpdate();

            // Panels/driver telemetry
            panelsTelemetry.addData("Path State", pathState);
            panelsTelemetry.addData("X", follower.getPose().getX());
            panelsTelemetry.addData("Y", follower.getPose().getY());
            panelsTelemetry.addData("Heading", follower.getPose().getHeading());
            panelsTelemetry.addData("Alliance Color", "BLUE");
            panelsTelemetry.update(telemetry);
            telemetry.addData("Alliance Color Saved", "BLUE");

            telemetry.update();
        }
    }

    /**
     * Inner class defining all the Pedro paths.
     */
    public static class Paths {
        public PathChain ShootingPosition1;
        public PathChain IntakeBallSet1;
        public PathChain OpenGate;
        public PathChain ShootingPosition2;
        public PathChain IntakePos2;
        public PathChain IntakeBallSet2;
        public PathChain ShootingPosition3;
        public PathChain IntakePos3;
        public PathChain IntakeBallSet3;
        public PathChain Parking;

        public Paths(Follower follower) {
            ShootingPosition1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.000, 135.000),

                                    new Pose(57.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            IntakeBallSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.000, 84.000),

                                    new Pose(28.000, 84.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            OpenGate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(28.000, 84.000),

                                    new Pose(18.121, 70.437)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                    .build();

            ShootingPosition2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.121, 70.437),

                                    new Pose(57.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            IntakePos2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.000, 84.000),

                                    new Pose(44.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            IntakeBallSet2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(44.000, 60.000),

                                    new Pose(28.000, 60.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            ShootingPosition3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(28.000, 60.000),

                                    new Pose(57.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            IntakePos3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.000, 84.000),

                                    new Pose(44.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            IntakeBallSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(44.000, 36.000),

                                    new Pose(28.000, 36.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Parking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(28.000, 36.000),

                                    new Pose(57.000, 120.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

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
                shootingFSM.start(getRuntime(), ShootingFSM.PowerLevel.CLOSE);
                if (pathTimer.getElapsedTimeSeconds() > SHOTGUN_SPINUP_DELAY) {
                    setPathState(pathState + 1);
                }
                break;
            case 1:

                shootingFSM.update(getRuntime(), telemetry);
                telemetry.addLine("Case " + pathState + ": Start Path1");
                follower.setMaxPower(PATH_POWER_STANDARD);
                follower.followPath(paths.ShootingPosition1);
                setPathState(pathState + 1);
                break;

            case 2:
                telemetry.addLine("Case " + pathState + ": Wait for Path1 and camera, then start read AprilTag");

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > SHOOT_TRIPLE_TIMEOUT) {
                    telemetry.addLine("Case " + pathState + ": exiting");
                    setPathState(pathState + 1);
                }
                break;

            case 3:
                telemetry.addLine("Case " + pathState + ": Read AprilTag then start Path2");

                intakeFSM.startIntaking();
                follower.setMaxPower(PATH_POWER_SLOW); //slow down for pickup

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {
                    telemetry.addLine("Case " + pathState + ": exiting");
                    follower.followPath(paths.IntakeBallSet1);
                    setPathState(pathState + 1);
                }
                break;

            case 4:
                telemetry.addLine("Case " + pathState + ": Wait for Path2, then shoot artifact");

                intakeFSM.updateIntaking(getRuntime(), true, telemetry);

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT * .25) {
                    telemetry.addLine("Case " + pathState + ": exiting");
                    follower.followPath(paths.OpenGate);
                    setPathState(pathState + 1);
                }
                break;

            case 5:
                telemetry.addLine("Case " + pathState + ": updateShooting...");
                shootingFSM.start(getRuntime(), ShootingFSM.PowerLevel.CLOSE);
                follower.setMaxPower    (PATH_POWER_STANDARD); //speed up again
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > SHOOT_TRIPLE_TIMEOUT) {
                    // now continue with next path
                    follower.followPath(paths.ShootingPosition2, true);
                    setPathState(pathState + 1);
                }
                break;

            case 6: //apples
                telemetry.addLine("Case " + pathState + ": Wait for Path3, then start Path4");

                shootingFSM.update(getRuntime(), telemetry);
                intakeFSM.startIntaking();

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {
                    telemetry.addLine("Case " + pathState + ": Move forward to pick up artifact 1p");

                    follower.setMaxPower(PATH_POWER_SLOW); //slow down for pickup

                    follower.followPath(paths.IntakeBallSet2, true);
                    setPathState(pathState + 1);
                }
                break;

            case 7:
                telemetry.addLine("Case " + pathState + ": Wait for Path7 to get into position, then start Path8");
                follower.setMaxPower(PATH_POWER_STANDARD);// resume normal speed

                intakeFSM.updateIntaking(getRuntime(),true, telemetry);
                shootingFSM.start(getRuntime(), ShootingFSM.PowerLevel.CLOSE);

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT * .25) {
                    telemetry.addLine("Case " + pathState + ": exiting");
                    follower.followPath(paths.ShootingPosition3);
                    setPathState(pathState + 1);

                }
                break;

            case 8:
                shootingFSM.update(getRuntime(), telemetry);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {
                    follower.followPath(paths.Parking);
                }
                break;


            case 9:
                telemetry.addLine("Case " + pathState + ": Wait for Path6 to finish, then stop");
                //shootPatternFSM.updateShootPattern(getRuntime());
                if (shootPatternFSM.isShootPatternDone()) {
                    telemetry.addLine("Case " + pathState + ": Done, setting state -1");
                    //rubberBands.setPower(0);

                    // Save final odometry position to SharedPreferences for TeleOp
                    SharedPreferences prefs = AppUtil.getInstance().getActivity().getSharedPreferences("ftc_prefs", android.content.Context.MODE_PRIVATE);
                    prefs.edit()
                            .putFloat("auto_final_x", (float) follower.getPose().getX())
                            .putFloat("auto_final_y", (float) follower.getPose().getY())
                            .putFloat("auto_final_heading", (float) follower.getPose().getHeading())
                            .apply();

                    telemetry.addData("Saved Odometry", String.format("X=%.1f, Y=%.1f, H=%.1f°",
                            follower.getPose().getX(),
                            follower.getPose().getY(),
                            Math.toDegrees(follower.getPose().getHeading())));

                    setPathState(pathState + 1); // done
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