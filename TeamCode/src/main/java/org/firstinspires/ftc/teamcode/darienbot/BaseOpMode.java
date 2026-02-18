package org.firstinspires.ftc.teamcode.darienbot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.darienbot.util.PoseStorage;

public abstract class BaseOpMode extends LinearOpMode {
    protected Robot robot;
    protected Scheduler scheduler;

    public void initRobot() {
        telemetry.addLine("Initializing robot...");
        telemetry.update();

        robot = new Robot();
        robot.init(hardwareMap);

        scheduler = new Scheduler();

        // Restore previous pose if needed
        PoseStorage.loadPoseIfNeeded(robot.drive);

        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }

    /**
     * Template method used by all child opmodes.
     */
    @Override
    public void runOpMode() throws InterruptedException {

        // Perform all initialization in one place
        initRobot();

        // Child classes will implement the rest of runOpMode() AFTER this returns
        // Example: waitForStart(), scheduling commands, running loops, etc.
    }

}
