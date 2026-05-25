package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team.fsm.AprilTagDetectionFSM;
import org.firstinspires.ftc.teamcode.team.fsm.TurretFSM;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

/**
 * Coordinates alliance target selection and camera-driven turret alignment.
 */
public class TurretVisionCoordinator {

    public static class AllianceSwitchResult {
        public final String alliance;
        public final boolean changed;

        public AllianceSwitchResult(String alliance, boolean changed) {
            this.alliance = alliance;
            this.changed = changed;
        }
    }

    private final AprilTagDetectionFSM tagFSM;
    private final TurretFSM turretFSM;
    private final int aprilTagIdGoalBlue;
    private final int aprilTagIdGoalRed;

    private boolean isReadingAprilTag = false;
    private int targetGoalTagId;

    public TurretVisionCoordinator(
            AprilTagDetectionFSM tagFSM,
            TurretFSM turretFSM,
            int aprilTagIdGoalBlue,
            int aprilTagIdGoalRed
    ) {
        this.tagFSM = tagFSM;
        this.turretFSM = turretFSM;
        this.aprilTagIdGoalBlue = aprilTagIdGoalBlue;
        this.aprilTagIdGoalRed = aprilTagIdGoalRed;
        this.targetGoalTagId = aprilTagIdGoalRed;
    }

    public void setAlliance(String alliance) {
        if ("BLUE".equals(alliance)) {
            targetGoalTagId = aprilTagIdGoalBlue;
            turretFSM.setOffsetBlue();
        } else if ("RED".equals(alliance)) {
            targetGoalTagId = aprilTagIdGoalRed;
            turretFSM.setOffsetRed();
        }
    }

    public AllianceSwitchResult handleAllianceButtons(
            boolean redPressed,
            boolean bluePressed,
            String currentAlliance,
            Telemetry telemetry
    ) {
        if (isReadingAprilTag) {
            return new AllianceSwitchResult(currentAlliance, false);
        }

        if (redPressed) {
            setAlliance("RED");
            telemetry.addLine("ALLIANCE SET TO RED!");
            return new AllianceSwitchResult("RED", true);
        }

        if (bluePressed) {
            setAlliance("BLUE");
            telemetry.addLine("ALLIANCE SET TO BLUE!");
            return new AllianceSwitchResult("BLUE", true);
        }

        return new AllianceSwitchResult(currentAlliance, false);
    }

    public void startReadingGoalId(double currentTime) {
        tagFSM.start(currentTime);
        isReadingAprilTag = true;
    }

    public void updateCameraControl(double currentTime, Telemetry telemetry) {
        if (turretFSM.getState() != TurretFSM.TurretStates.CAMERA) {
            return;
        }

        if (!isReadingAprilTag) {
            startReadingGoalId(currentTime);
            return;
        }

        tagFSM.update(currentTime, true, telemetry);
        telemetry.addLine("Goal Detection: Reading...");

        if (tagFSM.isDone()) {
            telemetry.addLine("Goal Detection: DONE reading!");
            isReadingAprilTag = false;

            ArrayList<AprilTagDetection> detections = tagFSM.getDetections();
            if (detections == null) {
                return;
            }

            if (targetGoalTagId == aprilTagIdGoalRed) {
                detections.removeIf(tag -> tag.id == aprilTagIdGoalBlue || tag.id == 21 || tag.id == 22 || tag.id == 23);
                turretFSM.setOffsetRed();
            } else if (targetGoalTagId == aprilTagIdGoalBlue) {
                detections.removeIf(tag -> tag.id == aprilTagIdGoalRed || tag.id == 21 || tag.id == 22 || tag.id == 23);
                turretFSM.setOffsetBlue();
            }

            if (!detections.isEmpty()) {
                telemetry.addLine("Goal Detection: FOUND APRILTAG!");
                AprilTagDetection detection = detections.get(0);
                if (detection.id == targetGoalTagId && detection.ftcPose != null) {
                    telemetry.addLine("Goal Detection: ALIGNING TURRET TO GOAL " + targetGoalTagId);
                    turretFSM.alignToBearing(detection.ftcPose.bearing);
                } else if (detection.ftcPose == null) {
                    telemetry.addLine("Goal Detection: WARNING - Pose estimation failed!");
                }
            }
        }
    }

    public int getTargetGoalTagId() {
        return targetGoalTagId;
    }
}


