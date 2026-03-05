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

    public static double PATH_POWER_STANDARD = 0.8;
    public static double PATH_POWER_SLOW = 0.4;


    public static double INTAKE_RUBBER_BANDS_DELAY = 0.2;
    public static double BALL_INTAKE_DELAY = 0;
    public static double SHOTGUN_SPINUP_DELAY = 0;
    public static double STANDARD_PATH_TIMEOUT = 2.0;
    public static double SHOOT_TRIPLE_TIMEOUT = 4.0;
    public static double TURRET_ROTATE_DELAY = 4;
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
        panelsTelemetry.update(telemetry);

        telemetry.addLine("BlueAudienceSidePedro: READY");
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
        // Constantly run top roller in intake mode


        // --- MAIN AUTONOMOUS LOOP ---
        while (opModeIsActive() && !isStopRequested()) {

            // Pedro follower must be updated every loop
            follower.update();

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
        public PathChain IntakePos1;
        public PathChain IntakeBallSet1;
        public PathChain ShootingPosition2;
        public PathChain IntakePos2;
        public PathChain IntakeBallSet2;
        public PathChain OpenGate;
        public PathChain ShootingPosition3;
        public PathChain IntakePos3;
        public PathChain IntakeBallSet3;
        public PathChain ShootingPosition4;
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

                                    new Pose(42.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            IntakeBallSet2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.000, 60.000),

                                    new Pose(22.000, 60.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            OpenGate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.000, 60.000),

                                    new Pose(19.000, 70.000)
                    )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                    .build();

            ShootingPosition3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.000, 70.000),

                                    new Pose(57.000, 18.000)
                    )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                    .build();

            IntakePos3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.000, 18.000),

                                    new Pose(42.000, 84.000)
                    )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            IntakeBallSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.000, 84.000),

                                    new Pose(22.000, 84.000)
                    )
                    ).setTangentHeadingInterpolation()

                    .build();

            ShootingPosition4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.000, 84.000),

                                    new Pose(57.000, 18.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

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
                // move to shooting position 1
                follower.setMaxPower(PATH_POWER_STANDARD);
                intakeFSM.setModeFull();
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
                // when shooting is done, Intake ballset 1
                shootingFSM.update(getRuntime(), telemetry);
                telemetry.addLine("Case " + pathState + ": Start IntakeBallSet1");
                if (!shootingFSM.isBusy() || pathTimer.getElapsedTimeSeconds() > SHOOT_TRIPLE_TIMEOUT) {
                    follower.followPath(paths.IntakePos1);
                    setPathState(pathState + 1);
                }
                break;

            case 3:
                telemetry.addLine("Case " + pathState + ": Read AprilTag then start Path2");
                shootingFSM.reset();
                if (!shootingFSM.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {
                    follower.setMaxPower(PATH_POWER_SLOW);
                    intakeFSM.startIntaking();
                    follower.followPath(paths.IntakeBallSet1);
                    setPathState(pathState + 1);
                }
                break;

            case 4:
                telemetry.addLine("Case " + pathState + ": Wait for Path2, then shoot artifact");
                intakeFSM.updateIntaking(getRuntime(), true, telemetry);
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {
                    telemetry.addLine("Case " + pathState + ": exiting");
                    follower.setMaxPower(PATH_POWER_STANDARD); //speed up again
                    follower.followPath(paths.ShootingPosition2, true);
                    setPathState(pathState + 1);
                }
                break;

            case 5:
                shootingFSM.start(getRuntime(), ShootingFSM.PowerLevel.FAR);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > TURRET_ROTATE_DELAY) {
                    setPathState(pathState + 1);
                }
                break;

            case 6:
                telemetry.addLine("Case " + pathState + ": updateShooting...");

                shootingFSM.update(getRuntime(), telemetry);
                if (!shootingFSM.isBusy() || pathTimer.getElapsedTimeSeconds() > SHOOT_TRIPLE_TIMEOUT) {
                    follower.followPath(paths.Parking, true);
                    shootingFSM.reset();
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