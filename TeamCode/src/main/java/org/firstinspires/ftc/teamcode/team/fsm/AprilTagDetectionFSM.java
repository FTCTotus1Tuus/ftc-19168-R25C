package org.firstinspires.ftc.teamcode.team.fsm;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class AprilTagDetectionFSM {
    private final AprilTagProcessor aprilTag;
    private final double timeoutSeconds;
    private boolean reading = false;
    private double startTime = 0;
    private ArrayList<AprilTagDetection> detections = null;

    public AprilTagDetectionFSM(AprilTagProcessor aprilTag, double timeoutSeconds) {
        this.aprilTag = aprilTag;
        this.timeoutSeconds = timeoutSeconds;
    }

    public void start(double currentTime) {
        reading = true;
        startTime = currentTime;
        detections = null;
    }

    public boolean update(double currentTime, boolean debug, Telemetry telemetry) {
        if (!reading) return false;
        ArrayList<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double elapsed = currentTime - startTime;
        if (!currentDetections.isEmpty() || elapsed >= timeoutSeconds) {
            // Create a defensive copy to prevent ConcurrentModificationException
            detections = new ArrayList<>(currentDetections);
            if (debug) {
                telemetryAprilTag(telemetry);
            }
            reading = false;
            return true;
        }
        return false;
    }

    public ArrayList<AprilTagDetection> getDetections() {
        return detections;
    }

    public boolean isDone() {
        return !reading;
    }

    protected void telemetryAprilTag(Telemetry telemetry) {

        telemetry.addData("# AprilTags Detected", detections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.ftcPose != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
    }

}
