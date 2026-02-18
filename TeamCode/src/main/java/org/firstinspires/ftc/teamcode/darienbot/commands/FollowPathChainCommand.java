package org.firstinspires.ftc.teamcode.darienbot.commands;

import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.darienbot.subsystems.DriveSubsystem;

public class FollowPathChainCommand implements Command {

    private final DriveSubsystem drive;
    private final PathChain pathChain;
    private boolean started = false;

    public FollowPathChainCommand(DriveSubsystem drive, PathChain pathChain) {
        this.drive = drive;
        this.pathChain = pathChain;
    }

    @Override
    public void init() {
        // Start following the chain
        drive.followPathChain(pathChain);
        started = true;
    }

    @Override
    public void update() {
        // nothing special; drive.update() is called in robot.update()
    }

    @Override
    public boolean isFinished() {
        // rely on follower's busy state
        return started && drive.isPathFinished();
    }

    @Override
    public void end() {
        // optional: you could call drive.stop() if you want
    }
}
