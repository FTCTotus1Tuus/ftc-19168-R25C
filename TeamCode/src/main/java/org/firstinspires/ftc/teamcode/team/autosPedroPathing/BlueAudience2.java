package org.firstinspires.ftc.teamcode.team.autosPedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.team.auto.AutoPlan;
import org.firstinspires.ftc.teamcode.team.auto.CleanupStep;
import org.firstinspires.ftc.teamcode.team.auto.FollowPathStep;
import org.firstinspires.ftc.teamcode.team.auto.GateCloseStep;
import org.firstinspires.ftc.teamcode.team.auto.IntakeStep;
import org.firstinspires.ftc.teamcode.team.auto.ShootSequenceStep;
import org.firstinspires.ftc.teamcode.team.auto.ShotgunSpinFarStep;
import org.firstinspires.ftc.teamcode.team.auto.StopShotgunStep;
import org.firstinspires.ftc.teamcode.team.auto.BlueAudienceSidePaths;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootingFSM;

/**
 * Pedro Pathing auto using LinearOpMode via DarienOpModeFSM.
 */

@Autonomous(name = "Blue Audience 9", group = "Pedro:Blues", preselectTeleOp = "TeleopFSM")
@Configurable
@Config
public class BlueAudience2 extends DarienOpModeFSM {

    // Tuning constants
    public static double STARTING_POSE_X = BlueAudienceSidePaths.STARTING_POSE_X;
    public static double STARTING_POSE_Y = BlueAudienceSidePaths.STARTING_POSE_Y;
    public static double STARTING_POSE_H_DEG = BlueAudienceSidePaths.STARTING_POSE_H_DEG;
    public static double PATH_POWER_STANDARD = 0.8;
    public static double PATH_POWER_SLOW = 0.4;
    public static double STANDARD_PATH_TIMEOUT = 2.0;
    public static double LONG_PATH_TIMEOUT = 4.0;
    public static double SHOOT_TRIPLE_TIMEOUT = 7.0;

    private AutoPlan autoPlan;
    private Timer planTimer;
    public double targetGoalX = DarienOpModeFSM.GOAL_BLUE_X;
    public double targetGoalY = DarienOpModeFSM.GOAL_BLUE_Y;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- ROBOT + HARDWARE INIT ---
        initControls();

        // --- PEDRO + TIMER INIT ---
        planTimer = new Timer();

        TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Starting pose
        follower.setStartingPose(new Pose(STARTING_POSE_X, STARTING_POSE_Y, Math.toRadians(STARTING_POSE_H_DEG)));

        // --- BUILD AUTO PLAN (three intake cycles, FAR distance) ---
        autoPlan = new AutoPlan()
                .add(new ShotgunSpinFarStep())
                .add(new FollowPathStep(BlueAudienceSidePaths.buildShootingPosition1(follower), PATH_POWER_STANDARD, STANDARD_PATH_TIMEOUT))
                .add(new ShootSequenceStep(ShootingFSM.PowerLevel.FAR, SHOOT_TRIPLE_TIMEOUT))
                .add(new GateCloseStep())
                .add(new FollowPathStep(BlueAudienceSidePaths.buildIntakePos1(follower), PATH_POWER_STANDARD, STANDARD_PATH_TIMEOUT))
                .add(new IntakeStep(BlueAudienceSidePaths.buildIntakeBallSet1(follower), PATH_POWER_SLOW, STANDARD_PATH_TIMEOUT))
                .add(new FollowPathStep(BlueAudienceSidePaths.buildShootingPosition2(follower), PATH_POWER_STANDARD, STANDARD_PATH_TIMEOUT))
                .add(new GateCloseStep())
                .add(new FollowPathStep(BlueAudienceSidePaths.buildIntakePos2(follower), PATH_POWER_STANDARD, STANDARD_PATH_TIMEOUT))
                .add(new IntakeStep(BlueAudienceSidePaths.buildIntakeBallSet2(follower), PATH_POWER_SLOW, STANDARD_PATH_TIMEOUT))
                .add(new FollowPathStep(BlueAudienceSidePaths.buildShootingPosition3(follower), PATH_POWER_STANDARD, LONG_PATH_TIMEOUT))
                .add(new ShootSequenceStep(ShootingFSM.PowerLevel.FAR, SHOOT_TRIPLE_TIMEOUT))
                .add(new FollowPathStep(BlueAudienceSidePaths.buildParking(follower), PATH_POWER_STANDARD, STANDARD_PATH_TIMEOUT))
                .add(new StopShotgunStep())
                .add(new CleanupStep());

        panelsTelemetry.debug("Status", "Initialized");
        telemetry.addLine("BlueAudience2: READY");
        panelsTelemetry.update(telemetry);

        turretFSM.center();

        // Save alliance color to shared preferences for TeleOp
        preferencesService.saveAutoAlliance("BLUE");
        telemetry.addLine("Alliance Color: BLUE (Saved to Preferences)");

        // --- WAIT FOR START ---
        waitForStart();
        if (isStopRequested()) return;

        autoPlan.init(this);
        targetGoalId = APRILTAG_ID_GOAL_BLUE;

        // --- MAIN AUTONOMOUS LOOP ---
        while (opModeIsActive() && !isStopRequested()) {

            // Pedro follower must be updated every loop
            follower.update();

            double robotX = follower.getPose().getX();
            double robotY = follower.getPose().getY();
            double robotHeadingRadians = follower.getPose().getHeading();

            turretFSM.setPositionFromOdometry(targetGoalX, targetGoalY, robotX, robotY, robotHeadingRadians);

            // Save final odometry position to SharedPreferences for TeleOp
            preferencesService.saveAutoFinalPose(
                    (float) robotX,
                    (float) robotY,
                    (float) robotHeadingRadians
            );

            // Drive the autonomous plan
            autoPlan.update(this);

            // Telemetry
            panelsTelemetry.addData("Plan Status", autoPlan.getStatus());
            panelsTelemetry.addData("X", robotX);
            panelsTelemetry.addData("Y", robotY);
            panelsTelemetry.addData("Heading", robotHeadingRadians);
            panelsTelemetry.addData("Alliance Color", "BLUE");
            addTraceTelemetry("Auto-BlueAudience2", autoPlan.getStatus(), planTimer.getElapsedTimeSeconds());
            displayRpmTelemetry();
            panelsTelemetry.update(telemetry);

            if (autoPlan.isComplete()) {
                break;
            }
        }

        stopRobot();
    }
}