package org.firstinspires.ftc.teamcode.team.core;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team.config.TurretConfig;
import org.firstinspires.ftc.teamcode.team.fsm.TurretFSM;
import org.firstinspires.ftc.teamcode.team.services.AprilTagService;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

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

    private final AprilTagService aprilTagService;
    private final TurretFSM turretFSM;
    private final int aprilTagIdGoalBlue;
    private final int aprilTagIdGoalRed;
    private final double cameraFallbackTimeoutMs;

    private boolean isReadingAprilTag = false;
    private boolean fallbackToOdometryActive = false;
    private double lastCameraLockTimeSec = Double.NaN;
    private double lastCameraAgeMs = Double.POSITIVE_INFINITY;
    private int lastDetectionCount = 0;
    private String lastVisionStatusLine = "CAMERA: idle";
    private String lastFallbackStatusLine = "ODOMETRY: no lock yet";
    private int targetGoalTagId;

    public TurretVisionCoordinator(
            AprilTagService aprilTagService,
            TurretFSM turretFSM,
            int aprilTagIdGoalBlue,
            int aprilTagIdGoalRed,
            double cameraFallbackTimeoutMs
    ) {
        this.aprilTagService = aprilTagService;
        this.turretFSM = turretFSM;
        this.aprilTagIdGoalBlue = aprilTagIdGoalBlue;
        this.aprilTagIdGoalRed = aprilTagIdGoalRed;
        this.cameraFallbackTimeoutMs = cameraFallbackTimeoutMs;
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
        aprilTagService.start(currentTime);
        isReadingAprilTag = true;
    }

    public void updateCameraControl(
            double currentTime,
            String autoAlliance,
            double robotX,
            double robotY,
            double robotHeadingRadians,
            Telemetry telemetry
    ) {
        if (turretFSM.getState() != TurretFSM.TurretStates.CAMERA) {
            if (aprilTagService.isReading()) {
                aprilTagService.stop(currentTime);
            }
            isReadingAprilTag = false;
            fallbackToOdometryActive = false;
            lastCameraAgeMs = Double.POSITIVE_INFINITY;
            lastDetectionCount = 0;
            lastVisionStatusLine = "MANUAL: camera control disabled";
            lastFallbackStatusLine = "MANUAL: no fallback active";
            return;
        }

        if (!isReadingAprilTag) {
            startReadingGoalId(currentTime);
        }

        AprilTagService.Snapshot snapshot = aprilTagService.poll(currentTime);
        lastDetectionCount = snapshot.getDetections().size();
        lastVisionStatusLine = String.format(
                "status=%s detections=%d target=%d",
                snapshot.getStatus(),
                lastDetectionCount,
                targetGoalTagId
        );
        telemetry.addData("Goal Detection", lastVisionStatusLine);

        if (snapshot.isDone()) {
            isReadingAprilTag = false;

            AprilTagDetection targetDetection = findTargetDetection(snapshot.getDetections());
            if (targetDetection != null && targetDetection.ftcPose != null) {
                turretFSM.alignToBearing(targetDetection.ftcPose.bearing);
                lastCameraLockTimeSec = currentTime;
                fallbackToOdometryActive = false;
                lastFallbackStatusLine = String.format("CAMERA: lock tag %d bearing %.1f", targetDetection.id, targetDetection.ftcPose.bearing);
                telemetry.addData("Goal Detection", "camera lock on tag %d", targetDetection.id);
            } else {
                telemetry.addLine("Goal Detection: no valid target tag pose");
                lastFallbackStatusLine = snapshot.getDetections().isEmpty()
                        ? "ODOMETRY: no detections"
                        : "ODOMETRY: no valid target pose";
            }
        }

        lastCameraAgeMs = Double.isNaN(lastCameraLockTimeSec)
                ? Double.POSITIVE_INFINITY
                : (currentTime - lastCameraLockTimeSec) * 1000.0;

        if (lastCameraAgeMs > cameraFallbackTimeoutMs) {
            fallbackToOdometryActive = true;
            lastFallbackStatusLine = Double.isInfinite(lastCameraAgeMs)
                    ? "ODOMETRY: no camera lock yet"
                    : String.format("ODOMETRY: stale lock %.0fms", lastCameraAgeMs);
            applyOdometryFallback(autoAlliance, robotX, robotY, robotHeadingRadians);
            telemetry.addData("Turret Camera Fallback", lastFallbackStatusLine);
        } else {
            lastFallbackStatusLine = String.format("CAMERA: fresh lock %.0fms", lastCameraAgeMs);
            telemetry.addData("Turret Camera Fallback", lastFallbackStatusLine);
        }
    }

    private AprilTagDetection findTargetDetection(List<AprilTagDetection> detections) {
        if (detections == null || detections.isEmpty()) {
            return null;
        }

        for (AprilTagDetection detection : detections) {
            if (detection.id == targetGoalTagId) {
                return detection;
            }
        }

        return null;
    }

    private void applyOdometryFallback(String autoAlliance, double robotX, double robotY, double robotHeadingRadians) {
        if ("RED".equals(autoAlliance)) {
            turretFSM.setOffsetRed();
            turretFSM.setPositionFromOdometry(TurretConfig.GOAL_RED_X, TurretConfig.GOAL_RED_Y, robotX, robotY, robotHeadingRadians);
        } else {
            turretFSM.setOffsetBlue();
            turretFSM.setPositionFromOdometry(TurretConfig.GOAL_BLUE_X, TurretConfig.GOAL_BLUE_Y, robotX, robotY, robotHeadingRadians);
        }
    }

    public boolean isFallbackToOdometryActive() {
        return fallbackToOdometryActive;
    }

    public String getVisionStatusLine() {
        return lastVisionStatusLine;
    }

    public String getFallbackStatusLine() {
        return lastFallbackStatusLine;
    }

    public double getLastCameraAgeMs() {
        return lastCameraAgeMs;
    }

    public int getLastDetectionCount() {
        return lastDetectionCount;
    }

    public int getTargetGoalTagId() {
        return targetGoalTagId;
    }
}


