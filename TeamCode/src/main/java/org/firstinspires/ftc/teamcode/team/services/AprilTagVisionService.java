package org.firstinspires.ftc.teamcode.team.services;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

/**
 * Owns the full camera lifecycle: AprilTag processor creation, VisionPortal build,
 * manual exposure/gain profile application, and portal teardown.
 *
 * <p>Call order:
 * <ol>
 *   <li>{@link #initialize()} — builds processor + portal + applies tuned exposure profile</li>
 *   <li>{@link #getAprilTagService(double)} — get the lifecycle service for start/poll/stop</li>
 *   <li>{@link #close()} — teardown portal at OpMode stop</li>
 * </ol>
 */
public class AprilTagVisionService {

    public enum CameraHealth {
        NOT_STARTED,
        PORTAL_BUILT,
        STREAMING,
        EXPOSURE_SET,
        EXPOSURE_FAILED,
        BUILD_FAILED
    }

    private final DarienOpModeFSM opMode;
    private AprilTagProcessor aprilTagProcessor;
    private AprilTagService aprilTagService;
    private VisionPortal visionPortal;
    private CameraHealth health = CameraHealth.NOT_STARTED;
    private String healthDetail = "";

    public AprilTagVisionService(DarienOpModeFSM opMode) {
        this.opMode = opMode;
    }

    /**
     * Builds the AprilTag processor and VisionPortal, then applies the tuned manual
     * exposure profile from DarienOpModeFSM constants.
     *
     * <p>Safe to call during initControls() — blocks briefly until streaming or stop.
     */
    public void initialize() {
        try {
            aprilTagProcessor = new AprilTagProcessor.Builder().build();
            health = CameraHealth.PORTAL_BUILT;
            healthDetail = "AprilTag processor built";

            visionPortal = new VisionPortal.Builder()
                    .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTagProcessor)
                    .build();
            healthDetail = "VisionPortal built";
        } catch (Exception e) {
            health = (aprilTagProcessor != null) ? CameraHealth.PORTAL_BUILT : CameraHealth.BUILD_FAILED;
            healthDetail = "Camera unavailable; running odometry fallback only";
            visionPortal = null;
        }

        aprilTagService = null;
        applyTunedExposure(DarienOpModeFSM.APRILTAG_EXPOSURE_MS, DarienOpModeFSM.APRILTAG_GAIN);
    }

    /**
     * @deprecated Use {@link #initialize()} instead — this no longer builds the VisionPortal.
     *             Kept only for call-sites that have not been migrated yet.
     */
    @Deprecated
    public void initializeAprilTagProcessor() {
        initialize();
    }

    public AprilTagService getAprilTagService(double timeoutSeconds) {
        if (aprilTagProcessor == null) {
            aprilTagService = new AprilTagService(null, timeoutSeconds);
            return aprilTagService;
        }
        if (aprilTagService == null) {
            aprilTagService = new AprilTagService(aprilTagProcessor, timeoutSeconds);
        } else {
            aprilTagService.setTimeoutSeconds(timeoutSeconds);
        }
        return aprilTagService;
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public CameraHealth getHealth() {
        return health;
    }

    public String getHealthDetail() {
        return healthDetail;
    }

    public boolean isStreaming() {
        return visionPortal != null
                && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    /**
     * Cleanly stops the camera stream. Call at OpMode stop or when camera is no longer needed.
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
        health = CameraHealth.NOT_STARTED;
        healthDetail = "Portal closed";
    }

    // -------------------------------------------------------------------------
    // PRIVATE — camera control helpers
    // -------------------------------------------------------------------------

    private void applyTunedExposure(int exposureMs, int gain) {
        if (visionPortal == null) {
            healthDetail = "Camera unavailable; running odometry fallback only";
            return;
        }

        boolean ok = setManualExposure(exposureMs, gain);
        if (ok) {
            health = CameraHealth.EXPOSURE_SET;
            healthDetail = String.format("Exposure=%dms Gain=%d", exposureMs, gain);
        } else {
            health = (health == CameraHealth.PORTAL_BUILT) ? CameraHealth.EXPOSURE_FAILED : health;
            healthDetail = "Camera unavailable or exposure failed; running odometry fallback only";
        }
    }

    public boolean setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return false;
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            opMode.telemetry.addData("Camera", "Waiting for stream...");
            opMode.telemetry.update();
            while (!opMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                opMode.sleep(20);
            }
            opMode.telemetry.addData("Camera", "Ready");
            opMode.telemetry.update();
        }

        if (!opMode.isStopRequested()) {
            try {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    opMode.sleep(50);
                }
                exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
                opMode.sleep(20);

                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(gain);
                opMode.sleep(20);

                opMode.telemetry.addData("Camera Exposure", exposureMS + "ms");
                opMode.telemetry.addData("Camera Gain", gain);
                opMode.telemetry.update();
                return true;
            } catch (Exception e) {
                opMode.telemetry.addData("Camera Control Error", e.getMessage());
                opMode.telemetry.update();
                return false;
            }
        }

        return false;
    }

    public int[] getCameraSettings() {
        if (visionPortal == null) {
            return new int[]{0, 0, 0, 0};
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            opMode.telemetry.addData("Camera", "Waiting for stream...");
            opMode.telemetry.update();
            while (!opMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                opMode.sleep(20);
            }
        }

        if (!opMode.isStopRequested()) {
            try {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                int minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
                int maxExposure = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                int minGain = gainControl.getMinGain();
                int maxGain = gainControl.getMaxGain();

                return new int[]{minExposure, maxExposure, minGain, maxGain};
            } catch (Exception e) {
                opMode.telemetry.addData("Camera Settings Error", e.getMessage());
                opMode.telemetry.update();
                return new int[]{0, 0, 0, 0};
            }
        }

        return new int[]{0, 0, 0, 0};
    }
}

