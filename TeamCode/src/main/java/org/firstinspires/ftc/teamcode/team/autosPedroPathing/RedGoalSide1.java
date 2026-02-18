package org.firstinspires.ftc.teamcode.team.autosPedroPathing;

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

/**
 * Pedro Pathing auto using LinearOpMode via DarienOpModeFSM.
 */

@Autonomous(name = "Red Goal 6", group = "Pedro:Reds", preselectTeleOp = "TeleopFSM")
@Configurable
public class RedGoalSide1 extends DarienOpModeFSM {

    private TelemetryManager panelsTelemetry;   // Panels Telemetry instance
    public Follower follower;                   // Pedro Pathing follower instance
    private int pathState;                      // State machine state
    private Paths paths;                        // Paths
    private Timer pathTimer, opmodeTimer;

    public static double PATH_POWER_STANDARD = 1.0;
    public static double PATH_POWER_SLOW = 0.4;


    public static double INTAKE_RUBBER_BANDS_DELAY = 0.2;
    public static double BALL_INTAKE_DELAY = 1.15;
    public static double SHOTGUN_SPINUP_DELAY = 0.3;
    public static double STANDARD_PATH_TIMEOUT = 2.0;
    public static double SHOOT_TRIPLE_TIMEOUT = 7.0;
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
        follower.setStartingPose(new Pose(123.714, 124.378, Math.toRadians(126)));

        // Build all the paths once
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        telemetry.addLine("RedGoalSidePedro: READY");
        telemetry.update();

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

        // Set the initial tray position immediately.
        TrayServo.setPosition(TRAY_POS_1_SCORE);


        // --- MAIN AUTONOMOUS LOOP ---
        while (opModeIsActive() && !isStopRequested()) {

            // Pedro follower must be updated every loop
            follower.update();

            // Drive the state machine
            pathState = autonomousPathUpdate();


            // Panels/driver telemetry
            panelsTelemetry.addData("Tray Curr", currentTrayPosition);
            //panelsTelemetry.addData("Tray Targ", targetTrayPosition);
            panelsTelemetry.addData("Path State", pathState);
            panelsTelemetry.addData("X", follower.getPose().getX());
            panelsTelemetry.addData("Y", follower.getPose().getY());
            panelsTelemetry.addData("Heading", follower.getPose().getHeading());
            panelsTelemetry.addData("Alliance Color", "RED");
            panelsTelemetry.update(telemetry);
            telemetry.addData("Alliance Color Saved", "RED");

            //telemetry.update();
        }
    }

    /**
     * Inner class defining all the Pedro paths.
     */
    public static class Paths {

        public PathChain ReadAprilTag;
        public PathChain ShootingPosition1;
        public PathChain IntakePosition1;
        public PathChain IntakeBall1p;
        public PathChain IntakeBall2p;
        public PathChain IntakeBall3g;
        public PathChain ShootingPosition2;
        public PathChain Parking;

        // 67
        public Paths(Follower follower) {
            ReadAprilTag = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123.714, 124.378),

                                    new Pose(96.776, 96.443)
                    )
                    ).setLinearHeadingInterpolation(Math.toRadians(126), Math.toRadians(110))

                    .build();

            ShootingPosition1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.776, 96.443),

                                    new Pose(96.766, 96.443)
                    )
                    ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(40))

                    .build();

            IntakePosition1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(96.766, 96.443),
                                    new Pose(85.898, 89.916),
                                    new Pose(100.500, 84.305)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                    .build();

            IntakeBall1p = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(100.500, 84.305),

                                    new Pose(106.000, 84.305)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            IntakeBall2p = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(106.000, 84.305),

                                    new Pose(111.000, 84.305)
                    )
                    ).setTangentHeadingInterpolation()

                    .build();

            IntakeBall3g = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(111.000, 84.305),

                                    new Pose(116.000, 84.305)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            ShootingPosition2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(116.000, 84.305),

                                    new Pose(96.776, 96.443)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))

                    .build();

            Parking = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(96.776, 96.443),
                                    new Pose(90.348, 84.038),
                                    new Pose(97.111, 112.002)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(35))

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
                telemetry.addLine("Case " + pathState + ": Start Path1");

                // Set the initial tray position
                TrayServo.setPosition(TRAY_POS_1_SCORE);
                follower.setMaxPower(PATH_POWER_STANDARD);
                shootArtifactFSM.shotGun(SHOT_GUN_POWER_UP);
                follower.followPath(paths.ReadAprilTag);
                setPathState(pathState + 1);
                break;

            case 1:
                telemetry.addLine("Case " + pathState + ": Wait for Path1 and camera, then start read AprilTag");

                if ((!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT)) {

                    telemetry.addLine("Case " + pathState + ": exiting");
                    // Start AprilTag reading after path1 is done
                    tagFSM.start(getRuntime());

                    setPathState(pathState + 1);
                }
                break;

            case 2:
                telemetry.addLine("Case " + pathState + ": Read AprilTag then start Path2");

                tagFSM.update(getRuntime(), true, telemetry);

                if ((tagFSM.isDone()) || pathTimer.getElapsedTimeSeconds() > TIMEOUT_APRILTAG_DETECTION) {
                    aprilTagDetections = tagFSM.getDetections();

                    telemetry.addLine("Case " + pathState + ": exiting");
                    topIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                    follower.followPath(paths.ShootingPosition1);
                    setPathState(pathState + 1);
                }
                break;

            case 3:
                telemetry.addLine("Case " + pathState + ": Wait for Path2, then shoot artifact");
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT * .25) {
                    telemetry.addLine("Case " + pathState + ": exiting");
                    setPathState(pathState + 1);
                }
                break;

            case 4:
                telemetry.addLine("Case " + pathState + ": start shooting");

                shootPatternFSM.startShootPattern(aprilTagDetections, getRuntime(), SHOT_GUN_POWER_UP);

                if (pathTimer.getElapsedTimeSeconds() > SHOTGUN_SPINUP_DELAY) {
                    setPathState(pathState + 1);
                }
                break;

            case 5:
                telemetry.addLine("Case " + pathState + ": updateShooting...");
                shootPatternFSM.updateShootPattern(getRuntime());

                if (shootPatternFSM.isShootPatternDone() || pathTimer.getElapsedTimeSeconds() > SHOOT_TRIPLE_TIMEOUT) {

                    rubberBands.setPower(INTAKE_RUBBER_BANDS_POWER);
                    // topIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                    leftIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                    rightIntake.setPower(INTAKE_INTAKE_ROLLER_POWER);
                    TrayServo.setPosition(TRAY_POS_2_INTAKE);

                    // now continue with next path
                    follower.followPath(paths.IntakePosition1, true);
                    setPathState(pathState + 1);
                }
                break;

            case 6:
                telemetry.addLine("Case " + pathState + ": Wait for Path3, then start Path4");
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {
                    telemetry.addLine("Case " + pathState + ": Move forward to pick up artifact 1p");

                    follower.setMaxPower(PATH_POWER_SLOW); //slow down for pickup

                    follower.followPath(paths.IntakeBall1p, true);
                    setPathState(pathState + 1);
                }
                break;

            case 7:
                telemetry.addLine("Case " + pathState + ": Wait for Path4");
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > BALL_INTAKE_DELAY) {
                    telemetry.addLine("Case " + pathState + ": Move forward to pick up artifact 2p");

                    TrayServo.setPosition(TRAY_POS_1_INTAKE);

                    setPathState(pathState + 1);
                }
                break;

            case 8:
                //wait for tray to rotate before next pickup
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_RUBBER_BANDS_DELAY) {

                    follower.followPath(paths.IntakeBall2p, true);
                    setPathState(pathState + 1);
                }
                break;

            case 9:
                telemetry.addLine("Case " + pathState + ": Wait for Path5");
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > BALL_INTAKE_DELAY) {
                    telemetry.addLine("Case " + pathState + ": Move forward to pick up artifact 3g");

                    TrayServo.setPosition(TRAY_POS_3_INTAKE);
                    setPathState(pathState + 1);
                }
                break;

            case 10:
                //wait for tray to rotate before next pickup
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_RUBBER_BANDS_DELAY) {

                    follower.followPath(paths.IntakeBall3g, true);
                    setPathState(pathState + 1);
                }
                break;

            case 11:
                telemetry.addLine("Case " + pathState + ": Wait for Path6 to pick up artifact, then start Path7");

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > BALL_INTAKE_DELAY) {
                    telemetry.addLine("Case " + pathState + ": Moving to shooting position");
                    follower.setMaxPower(PATH_POWER_STANDARD);// resume normal speed

                    follower.followPath(paths.ShootingPosition2, true);
                    TrayServo.setPosition(TRAY_POS_2_SCORE);
                    rubberBands.setPower(0);
                    //topIntake.setPower(0);
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    shootArtifactFSM.shotGun(SHOT_GUN_POWER_UP);
                    setPathState(pathState + 1);
                }
                break;

            case 12:
                telemetry.addLine("Case " + pathState + ": Wait for Path7 to get into position, then start Path8");


                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > SHOTGUN_SPINUP_DELAY) {
                    telemetry.addLine("Case " + pathState + ": Shoot the pattern");

                    shootPatternFSM.startShootPattern(aprilTagDetections, getRuntime(), SHOT_GUN_POWER_UP);
                    setPathState(pathState + 1);
                }
                break;

            case 13:
                telemetry.addLine("Case " + pathState + ": Wait for Path6 to finish, then stop");
                shootPatternFSM.updateShootPattern(getRuntime());
                if (shootPatternFSM.isShootPatternDone()) {
                    telemetry.addLine("Case " + pathState + ": Done, setting state -1");
                    topIntake.setPower(0);
                    //rubberBands.setPower(0);
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