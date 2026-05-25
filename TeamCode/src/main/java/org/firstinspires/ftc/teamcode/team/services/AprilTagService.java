package org.firstinspires.ftc.teamcode.team.services;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Lifecycle-oriented AprilTag polling service.
 *
 * The service reads from an FTC AprilTagProcessor only during an active read window
 * and returns immutable snapshots so callers do not mutate shared lists.
 */
public class AprilTagService {

    public enum Status {
        IDLE,
        READING,
        COMPLETE,
        STOPPED
    }

    public static class Snapshot {
        private final Status status;
        private final double startTimeSec;
        private final double pollTimeSec;
        private final double elapsedTimeSec;
        private final double timeoutSec;
        private final List<AprilTagDetection> detections;

        public Snapshot(
                Status status,
                double startTimeSec,
                double pollTimeSec,
                double elapsedTimeSec,
                double timeoutSec,
                List<AprilTagDetection> detections
        ) {
            this.status = status;
            this.startTimeSec = startTimeSec;
            this.pollTimeSec = pollTimeSec;
            this.elapsedTimeSec = elapsedTimeSec;
            this.timeoutSec = timeoutSec;
            this.detections = detections;
        }

        public Status getStatus() {
            return status;
        }

        public double getStartTimeSec() {
            return startTimeSec;
        }

        public double getPollTimeSec() {
            return pollTimeSec;
        }

        public double getElapsedTimeSec() {
            return elapsedTimeSec;
        }

        public double getTimeoutSec() {
            return timeoutSec;
        }

        public List<AprilTagDetection> getDetections() {
            return detections;
        }

        public boolean isDone() {
            return status == Status.COMPLETE || status == Status.STOPPED;
        }

        public boolean isTimedOut() {
            return isDone() && detections.isEmpty() && elapsedTimeSec >= timeoutSec;
        }
    }

    private final AprilTagProcessor aprilTagProcessor;
    private double timeoutSeconds;

    private boolean reading = false;
    private double startTimeSec = 0;
    private Snapshot lastSnapshot;

    public AprilTagService(AprilTagProcessor aprilTagProcessor, double timeoutSeconds) {
        this.aprilTagProcessor = aprilTagProcessor;
        this.timeoutSeconds = timeoutSeconds;
        this.lastSnapshot = new Snapshot(
                Status.IDLE,
                0,
                0,
                0,
                timeoutSeconds,
                Collections.<AprilTagDetection>emptyList()
        );
    }

    public void setTimeoutSeconds(double timeoutSeconds) {
        this.timeoutSeconds = timeoutSeconds;
    }

    public void start(double currentTimeSec) {
        reading = true;
        startTimeSec = currentTimeSec;
        lastSnapshot = new Snapshot(
                Status.READING,
                startTimeSec,
                currentTimeSec,
                0,
                timeoutSeconds,
                Collections.<AprilTagDetection>emptyList()
        );
    }

    public Snapshot poll(double currentTimeSec) {
        if (!reading) {
            return lastSnapshot;
        }

        List<AprilTagDetection> immutableDetections = copyDetections(aprilTagProcessor.getDetections());
        double elapsedSec = currentTimeSec - startTimeSec;
        boolean done = !immutableDetections.isEmpty() || elapsedSec >= timeoutSeconds;

        Status status = done ? Status.COMPLETE : Status.READING;
        lastSnapshot = new Snapshot(
                status,
                startTimeSec,
                currentTimeSec,
                elapsedSec,
                timeoutSeconds,
                immutableDetections
        );

        if (done) {
            reading = false;
        }

        return lastSnapshot;
    }

    public void stop(double currentTimeSec) {
        reading = false;
        double elapsedSec = currentTimeSec - startTimeSec;
        lastSnapshot = new Snapshot(
                Status.STOPPED,
                startTimeSec,
                currentTimeSec,
                elapsedSec,
                timeoutSeconds,
                Collections.<AprilTagDetection>emptyList()
        );
    }

    public boolean isReading() {
        return reading;
    }

    public Snapshot getLastSnapshot() {
        return lastSnapshot;
    }

    private List<AprilTagDetection> copyDetections(List<AprilTagDetection> currentDetections) {
        if (currentDetections == null || currentDetections.isEmpty()) {
            return Collections.<AprilTagDetection>emptyList();
        }
        return Collections.unmodifiableList(new ArrayList<>(currentDetections));
    }
}

