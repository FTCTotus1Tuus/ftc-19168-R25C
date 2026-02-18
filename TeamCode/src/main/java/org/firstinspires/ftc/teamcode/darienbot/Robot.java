package org.firstinspires.ftc.teamcode.darienbot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.darienbot.subsystems.*;

public class Robot {
    public DriveSubsystem drive;
    public IntakeSubsystem intake;
    public TraySubsystem tray;
    public ScoringSubsystem scoring;

    public void init(HardwareMap hardwareMap) {
        drive = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        tray = new TraySubsystem(hardwareMap);
        scoring = new ScoringSubsystem(hardwareMap);
    }

    public void update() {
        drive.update();
        intake.update();
        tray.update();
        scoring.update();
    }

    public void stop() {
        drive.stop();
        scoring.stop();
        intake.stop();
        // tray has no motors, so no stop() needed there
    }
}
