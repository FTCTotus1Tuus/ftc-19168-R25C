package org.firstinspires.ftc.teamcode.darienbot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.darienbot.util.PoseStorage;
import org.firstinspires.ftc.teamcode.darienbot.util.UtilPanels;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DriveSubsystem {

    // HARDWARE COMPONENTS
    private final Follower follower;
    private Path activePath = null;
    private PathChain activePathChain = null;

    public DriveSubsystem(HardwareMap hw) {
        // Use the Constants factory method
        follower = Constants.createFollower(hw);
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public Follower getFollower() {
        return follower;
    }

    public void followPath(Path path) {
        activePath = path;
        follower.followPath(path);
    }

    public void followPathChain(PathChain chain) {
        activePathChain = chain;
        follower.followPath(chain);
    }

    public boolean isPathFinished() {
        return !follower.isBusy();
    }

    public void update() {
        follower.update();

        // Draw robot and path on Panels
        UtilPanels.draw(follower, activePath);

        // Store current pose for next opmode
        PoseStorage.storePose(getPose());
    }

    public void stop() {
        //follower.stop();        // optional, clean stop
        //setDrivePowers(0,0,0);  // make sure motors stop
    }

}
