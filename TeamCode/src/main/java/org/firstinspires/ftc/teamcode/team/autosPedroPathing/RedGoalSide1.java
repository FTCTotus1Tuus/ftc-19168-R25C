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
import org.firstinspires.ftc.teamcode.team.auto.ShotgunSpinStep;
import org.firstinspires.ftc.teamcode.team.auto.StopShotgunStep;
import org.firstinspires.ftc.teamcode.team.auto.RedGoalSidePaths;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootingFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShotgunFSM;

/**
 * Red Goal Side 1 - Autonomous with AutoPlan framework.
 * Sequence: Move to shoot → Shoot → Intake once → Park → Shoot again.
 */

@Autonomous(name = "Red Goal 6", group = "Pedro:Reds", preselectTeleOp = "TeleopFSM")
@Config
@Configurable
public class RedGoalSide1 extends DarienOpModeFSM {

    // Tuning constants
    public static double STARTING_POSE_X = RedGoalSidePaths.STARTING_POSE_X;
    public static double STARTING_POSE_Y = RedGoalSidePaths.STARTING_POSE_Y;
    public static double STARTING_POSE_H_DEG = RedGoalSidePaths.STARTING_POSE_H_DEG;
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

        // --- BUILD AUTO PLAN ---
        autoPlan = new AutoPlan()
                .add(new ShotgunSpinStep())
                .add(new FollowPathStep(RedGoalSidePaths.buildShootingPosition1(follower), PATH_POWER_STANDARD, STANDARD_PATH_TIMEOUT))
                .add(new ShootSequenceStep(ShootingFSM.PowerLevel.CLOSE, SHOOT_TRIPLE_TIMEOUT))
                .add(new GateCloseStep())
                .add(new FollowPathStep(RedGoalSidePaths.buildIntakePos1(follower), PATH_POWER_STANDARD, STANDARD_PATH_TIMEOUT))
                .add(new IntakeStep(RedGoalSidePaths.buildIntakeBallSet1(follower), PATH_POWER_SLOW, STANDARD_PATH_TIMEOUT))
                .add(new FollowPathStep(RedGoalSidePaths.buildParking1(follower), PATH_POWER_STANDARD, LONG_PATH_TIMEOUT))
                .add(new ShootSequenceStep(ShootingFSM.PowerLevel.CLOSE, SHOOT_TRIPLE_TIMEOUT))
                .add(new StopShotgunStep())
                .add(new CleanupStep());

        panelsTelemetry.debug("Status", "Initialized");
        telemetry.addLine("RedGoalSide1: READY");
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

            // Keep shotgun PID running during the plan
            double targetShotgunRPM = shotgunFSM.getState() == ShotgunFSM.State.POWER_UP_FAR
                ? DarienOpModeFSM.SHOT_GUN_POWER_UP_FAR_RPM_AUTO
                : DarienOpModeFSM.SHOT_GUN_POWER_UP_RPM_AUTO;
            shotgunFSM.toPowerUp(targetShotgunRPM);

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
            addTraceTelemetry("Auto-RedGoalSide1", autoPlan.getStatus(), planTimer.getElapsedTimeSeconds());
            displayRpmTelemetry();
            panelsTelemetry.update(telemetry);

            if (autoPlan.isComplete()) {
                break;
            }
        }

        stopRobot();
    }
}
