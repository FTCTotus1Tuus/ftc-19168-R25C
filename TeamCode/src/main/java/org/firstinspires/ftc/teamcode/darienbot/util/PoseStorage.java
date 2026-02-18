package org.firstinspires.ftc.teamcode.darienbot.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.darienbot.subsystems.DriveSubsystem;

public class PoseStorage {

    /**
     * Stores the last known pose; persists between opmodes during the same app run
     */
    public static Pose currentPose = new Pose();

    /**
     * Call this each loop to update the stored pose
     */
    public static void storePose(Pose pose) {
        currentPose = pose;
    }

    /**
     * Load last stored pose into the drive subsystem
     * Typically called during opmode init
     */
    public static void loadPoseIfNeeded(DriveSubsystem drive) {
        if (currentPose != null) {
            drive.setPose(currentPose);
        }
    }

    /**
     * Optional: clear stored pose
     */
    public static void clear() {
        currentPose = new Pose();
    }
}
