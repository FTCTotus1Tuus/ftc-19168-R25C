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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;


@Autonomous(name = "Red Audience 12", group = "Pedro:Reds", preselectTeleOp = "TeleopFSM")
@Configurable
@Config
public class RedAudience3 extends DarienOpModeFSM {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState;                      // State machine state
    private Paths paths;                        // Paths
    private Timer pathTimer, opmodeTimer;

    public static double PATH_POWER_STANDARD = 1;
    public static double PATH_POWER_SLOW = 0.25;
    //public static double SHOT_GUN_POWER_UP = 0.6*.9;

    public static double INTAKE_RUBBER_BANDS_DELAY = 0.2;
    public static double BALL_INTAKE_DELAY = 1.0;
    public static double SHOTGUN_SPINUP_DELAY = 1.0;
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
        follower.setStartingPose(new Pose(88, 9, Math.toRadians(90)));

        // Build all the paths once
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        telemetry.addLine("RedAudienceSidePedro: READY");
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
        //  TrayServo.setPosition(TRAY_POS_1_SCORE);
        // Constantly run top roller in intake mode
        topIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);

        // --- MAIN AUTONOMOUS LOOP ---
        while (opModeIsActive() && !isStopRequested()) {

            // Pedro follower must be updated every loop
            follower.update();

            // Drive the state machine
            pathState = autonomousPathUpdate();


            /*
            // Update tray servo FSM if running
            if (trayServoFSM.isRunning()) {
                trayServoFSM.update(getRuntime());
                if (!trayServoFSM.isRunning()) {
                    // Update current tray position when done
                    currentTrayPosition = targetTrayPosition;
                }
            }

             */

            // Panels/driver telemetry
            // panelsTelemetry.addData("Tray Curr", currentTrayPosition);
            //panelsTelemetry.addData("Tray Targ", targetTrayPosition);
            panelsTelemetry.addData("Path State", pathState);
            panelsTelemetry.addData("X", follower.getPose().getX());
            panelsTelemetry.addData("Y", follower.getPose().getY());
            panelsTelemetry.addData("Heading", follower.getPose().getHeading());
            panelsTelemetry.addData("Alliance Color", "RED");
            panelsTelemetry.update(telemetry);
            telemetry.addData("Alliance Color Saved", "RED");

            telemetry.update();
        }
    }

    /**
     * Inner class defining all the Pedro paths.
     */
    public static class Paths {
        public PathChain ShootingPosition;
        public PathChain IntakePosition;
        public PathChain IntakeBallA;
        public PathChain ShootingPosition2;
        public PathChain IntakePosition2;
        public PathChain IntakeBallB;
        public PathChain ShootingPosition3;
        public PathChain OpenGate;
        public PathChain IntakePosition3;
        public PathChain IntakeBallC;
        public PathChain ShootingPosition4;
        public PathChain Parking;

        public Paths(Follower follower) {
            ShootingPosition = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 9.000),

                                    new Pose(88.000, 18.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(63))

                    .build();

            IntakePosition = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 18.000),

                                    new Pose(102.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(63), Math.toRadians(0))

                    .build();

            IntakeBallA = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.000, 36.000),

                                    new Pose(128.000, 35.750)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ShootingPosition2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.000, 35.750),

                                    new Pose(88.000, 18.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(63))

                    .build();

            IntakePosition2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 18.000),

                                    new Pose(102.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(63), Math.toRadians(0))

                    .build();

            IntakeBallB = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.000, 60.000),

                                    new Pose(128.000, 59.500)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            ShootingPosition3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.000, 59.500),

                                    new Pose(88.000, 18.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(63))

                    .build();

            OpenGate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 18.000),

                                    new Pose(129.000, 72.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(63), Math.toRadians(-90))

                    .build();

            IntakePosition3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(129.000, 72.000),
                                    new Pose(96.000, 63.000),
                                    new Pose(99.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))

                    .build();

            IntakeBallC = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(99.000, 84.000),

                                    new Pose(128.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            ShootingPosition4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.000, 84.000),

                                    new Pose(88.000, 18.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(63))

                    .build();

            Parking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 18.000),

                                    new Pose(88.000, 31.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
    }


//Todo: fix angle for shooting
//67

    public int autonomousPathUpdate() {
        telemetry.addData("PathState", pathState);
        telemetry.addData("FollowerBusy", follower.isBusy());
        telemetry.addData("PathTimer", pathTimer.getElapsedTimeSeconds());

        switch (pathState) {
            case 0:
                //start reading april tags
                telemetry.addLine("Case " + pathState + ": Wait for Camera");

                // Set the initial tray position
                //TrayServo.setPosition(TRAY_POS_1_SCORE);
                tagFSM.start(getRuntime());
                follower.setMaxPower(PATH_POWER_STANDARD * .75); //normal speed
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {

                    telemetry.addLine("Case " + pathState + ": exiting");

                    setPathState(pathState + 1);
                }
                break;

            case 1:
                //once april tags done reading, move to shooting position 1
                telemetry.addLine("Case " + pathState + ":");

                tagFSM.update(getRuntime(), true, telemetry);
                follower.setMaxPower(PATH_POWER_STANDARD); //normal speed

                if ((tagFSM.isDone()) || pathTimer.getElapsedTimeSeconds() > TIMEOUT_APRILTAG_DETECTION) {
                    aprilTagDetections = tagFSM.getDetections();
                    aprilTagDetections.removeIf(tag -> tag.id == 20 || tag.id == 24);
                    follower.followPath(paths.ShootingPosition);

                    setPathState(pathState + 1);
                }
                break;

            case 2:
                //move to shooting position 1
                telemetry.addLine("Case " + pathState + ": wait for Path 1...");

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {
                    topIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                    setPathState(pathState + 1);
                }
                break;

            case 3:
                //once at shooting position 1, shoot artifacts set 1
                telemetry.addLine("Case " + pathState + ": start shooting...");

                shootPatternFSM.startShootPattern(aprilTagDetections, getRuntime(), SHOT_GUN_POWER_UP_FAR);

                if (pathTimer.getElapsedTimeSeconds() > SHOTGUN_SPINUP_DELAY) { // increased time to allow for motor to spin up
                    setPathState(pathState + 1);
                }
                break;

            case 4:
                //once artifacts set 1 shot, move to intake position
                telemetry.addLine("Case " + pathState + ": Update shooting...");

                shootPatternFSM.updateShootPattern(getRuntime());

                if (shootPatternFSM.isShootPatternDone() || pathTimer.getElapsedTimeSeconds() > SHOOT_TRIPLE_TIMEOUT) {

                    rubberBands.setPower(INTAKE_RUBBER_BANDS_POWER);
                    //  topIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                    leftIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                    rightIntake.setPower(INTAKE_INTAKE_ROLLER_POWER);
                    //setTrayPosition(TRAY_POS_3_INTAKE);
                    follower.followPath(paths.IntakePosition, true);
                    setPathState(pathState + 1);
                }
                break;

            case 5:
                //when in position, go to intake position 1
                telemetry.addLine("Case " + pathState + ": Going to intake position 1");

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {
                    follower.setMaxPower(PATH_POWER_SLOW); //slow down for pickup

                    follower.followPath(paths.IntakeBallA, true);
                    setPathState(pathState + 1);
                }
                break;

            case 6:
                //once ball 3p intaken, move to shooting position 2
                telemetry.addLine("Case " + pathState + ": Move to shoot position 2");

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) { // increased time to allow for motor to spin up
                    follower.setMaxPower(PATH_POWER_STANDARD); //reset to normal speed
                    //setTrayPosition(TRAY_POS_2_SCORE);

                    shootArtifactFSM.shotGun(SHOT_GUN_POWER_UP_FAR);
                    follower.followPath(paths.ShootingPosition2, true);
                    setPathState(pathState + 1);
                }
                break;

            case 7:
                //once at shooting position 2, shoot artifacts set 2
                telemetry.addLine("Case " + pathState + ": Wait for ShootingPosition, then shoot artifact");


                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > SHOTGUN_SPINUP_DELAY) {
                    shootPatternFSM.startShootPattern(aprilTagDetections, getRuntime(), SHOT_GUN_POWER_UP_FAR);
                    setPathState(pathState + 1);
                }
                break;

            case 8:
                //once artifacts set 2 shot, move to parking
                telemetry.addLine("Case " + pathState + ": Update shooting");

                shootPatternFSM.updateShootPattern(getRuntime());

                if (shootPatternFSM.isShootPatternDone() || pathTimer.getElapsedTimeSeconds() > SHOOT_TRIPLE_TIMEOUT) {
                  /*
                    rubberBands.setPower(0);
                    topIntake.setPower(0);
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                   */
                    //TrayServo.setPosition(TRAY_POS_2_INTAKE);
                    follower.followPath(paths.IntakePosition2, true);
                    setPathState(pathState + 1);
                }
                break;

            case 9:
                //when in position, go to intake position 1
                telemetry.addLine("Case " + pathState + ": Going to intake position 2");
                shootPatternFSM.startShootPattern(aprilTagDetections, getRuntime(), SHOT_GUN_POWER_UP_FAR); // keep updating shoot pattern to maintain shotgun spin

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {
                    follower.setMaxPower(PATH_POWER_SLOW); //slow down for pickup

                    follower.followPath(paths.IntakeBallB, true);
                    setPathState(pathState + 1);
                }
                break;

            case 10:
                //once ball 3p intaken, move to shooting position 2
                telemetry.addLine("Case " + pathState + ": Move to shoot position 2");
                shootPatternFSM.updateShootPattern(getRuntime());

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) { // increased time to allow for motor to spin up
                    follower.setMaxPower(PATH_POWER_STANDARD); //reset to normal speed
                    //setTrayPosition(TRAY_POS_2_SCORE);

                    shootArtifactFSM.shotGun(SHOT_GUN_POWER_UP_FAR);
                    follower.followPath(paths.ShootingPosition3, true);
                    setPathState(pathState + 1);
                }
                break;

            case 11:
                telemetry.addLine("Case " + pathState + ": Move to shoot position 2");

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) { // increased time to allow for motor to spin up


                    follower.followPath(paths.OpenGate, true);
                    setPathState(pathState + 1);
                }
                break;


            case 12:
                //once artifacts set 1 shot, move to intake position
                telemetry.addLine("Case " + pathState + ": Update shooting...");

                shootPatternFSM.updateShootPattern(getRuntime());

                if (shootPatternFSM.isShootPatternDone() || pathTimer.getElapsedTimeSeconds() > SHOOT_TRIPLE_TIMEOUT) {

                    rubberBands.setPower(INTAKE_RUBBER_BANDS_POWER);
                    //  topIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                    leftIntake.setPower(-INTAKE_INTAKE_ROLLER_POWER);
                    rightIntake.setPower(INTAKE_INTAKE_ROLLER_POWER);
                    //setTrayPosition(TRAY_POS_3_INTAKE);
                    follower.followPath(paths.IntakePosition3, true);
                    setPathState(pathState + 1);
                }
                break;

            case 13:
                //when in position, go to intake position 1
                telemetry.addLine("Case " + pathState + ": Going to intake position 1");

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) {
                    follower.setMaxPower(PATH_POWER_SLOW); //slow down for pickup

                    follower.followPath(paths.IntakeBallC, true);
                    setPathState(pathState + 1);
                }
                break;

            case 14:
                //once ball 3p intaken, move to shooting position 2
                telemetry.addLine("Case " + pathState + ": Move to shoot position 2");

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) { // increased time to allow for motor to spin up
                    follower.setMaxPower(PATH_POWER_STANDARD); //reset to normal speed
                    //setTrayPosition(TRAY_POS_2_SCORE);

                    shootArtifactFSM.shotGun(SHOT_GUN_POWER_UP_FAR);
                    follower.followPath(paths.ShootingPosition4, true);
                    setPathState(pathState + 1);
                }
                break;

            case 15:
                telemetry.addLine("Case " + pathState + ": Move to shoot position 2");

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > STANDARD_PATH_TIMEOUT) { // increased time to allow for motor to spin up


                    follower.followPath(paths.Parking, true);
                    setPathState(pathState + 1);
                }
                break;

            case 16:
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