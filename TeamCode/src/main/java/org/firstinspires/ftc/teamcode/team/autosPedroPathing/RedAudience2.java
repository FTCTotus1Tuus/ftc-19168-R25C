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
import org.firstinspires.ftc.teamcode.team.auto.RedAudienceSidePaths;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootingFSM;

/**
 * Red Audience Side 2 - Autonomous with AutoPlan framework.
 * Audience side variant with different intake cycle pattern.
 */

@Autonomous(name = "Red Audience 9", group = "Pedro:Reds", preselectTeleOp = "TeleopFSM")
@Configurable
@Config
public class RedAudience2 extends DarienOpModeFSM {

    // Tuning constants
    public static double STARTING_POSE_X = RedAudienceSidePaths.STARTING_POSE_X;
    public static double STARTING_POSE_Y = RedAudienceSidePaths.STARTING_POSE_Y;
    public static double STARTING_POSE_H_DEG = RedAudienceSidePaths.STARTING_POSE_H_DEG;
    public static double PATH_POWER_STANDARD = 0.8;
    public static double PATH_POWER_SLOW = 0.4;
    public static double STANDARD_PATH_TIMEOUT = 2.0;
    public static double LONG_PATH_TIMEOUT = 4.0;
    public static double SHOOT_TRIPLE_TIMEOUT = 7.0;

    private AutoPlan autoPlan;
    private Timer planTimer;
    public double targetGoalX = DarienOpModeFSM.GOAL_RED_X;
    public double targetGoalY = DarienOpModeFSM.GOAL_RED_Y;

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
                .add(new FollowPathStep(RedAudienceSidePaths.buildShootingPosition1(follower), PATH_POWER_STANDARD, STANDARD_PATH_TIMEOUT))
                .add(new ShootSequenceStep(ShootingFSM.PowerLevel.FAR, SHOOT_TRIPLE_TIMEOUT))
                .add(new GateCloseStep())
                .add(new FollowPathStep(RedAudienceSidePaths.buildIntakePos1(follower), PATH_POWER_STANDARD, STANDARD_PATH_TIMEOUT))
                .add(new IntakeStep(RedAudienceSidePaths.buildIntakeBallSet1(follower), PATH_POWER_SLOW, STANDARD_PATH_TIMEOUT))
                .add(new FollowPathStep(RedAudienceSidePaths.buildShootingPosition2(follower), PATH_POWER_STANDARD, STANDARD_PATH_TIMEOUT))
                .add(new GateCloseStep())
                .add(new FollowPathStep(RedAudienceSidePaths.buildIntakePos2(follower), PATH_POWER_STANDARD, STANDARD_PATH_TIMEOUT))
                .add(new IntakeStep(RedAudienceSidePaths.buildIntakeBallSet2(follower), PATH_POWER_SLOW, STANDARD_PATH_TIMEOUT))
                .add(new FollowPathStep(RedAudienceSidePaths.buildShootingPosition3(follower), PATH_POWER_STANDARD, LONG_PATH_TIMEOUT))
                .add(new ShootSequenceStep(ShootingFSM.PowerLevel.FAR, SHOOT_TRIPLE_TIMEOUT))
                .add(new FollowPathStep(RedAudienceSidePaths.buildParking(follower), PATH_POWER_STANDARD, STANDARD_PATH_TIMEOUT))
                .add(new StopShotgunStep())
                .add(new CleanupStep());

        panelsTelemetry.debug("Status", "Initialized");
        telemetry.addLine("RedAudience2: READY");
        panelsTelemetry.update(telemetry);

        turretFSM.center();

        // Save alliance color to shared preferences for TeleOp
        preferencesService.saveAutoAlliance("RED");
        telemetry.addLine("Alliance Color: RED (Saved to Preferences)");

        // --- WAIT FOR START ---
        waitForStart();
        if (isStopRequested()) return;

        autoPlan.init(this);
        targetGoalId = APRILTAG_ID_GOAL_RED;

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
            panelsTelemetry.addData("Alliance Color", "RED");
            addTraceTelemetry("Auto-RedAudience2", autoPlan.getStatus(), planTimer.getElapsedTimeSeconds());
            displayRpmTelemetry();
            panelsTelemetry.update(telemetry);

            if (autoPlan.isComplete()) {
                break;
            }
        }

        stopRobot();
    }
}