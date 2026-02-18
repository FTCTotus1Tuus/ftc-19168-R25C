package org.firstinspires.ftc.teamcode.darienbot.commands;

import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.darienbot.subsystems.DriveSubsystem;

public class FollowPathCommand implements Command {
    private final DriveSubsystem drive;
    private final Path path;

    public FollowPathCommand(DriveSubsystem drive, Path path) {
        this.drive = drive;
        this.path = path;
    }

    @Override
    public void init() {
        drive.followPath(path);
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return drive.isPathFinished();
    }

    @Override
    public void end() {
    }
}
