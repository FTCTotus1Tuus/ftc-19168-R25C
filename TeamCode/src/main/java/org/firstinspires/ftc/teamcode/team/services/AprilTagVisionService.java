package org.firstinspires.ftc.teamcode.team.services;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

/**
 * Owns AprilTag processor and camera control helpers for an OpMode lifecycle.
 */
public class AprilTagVisionService {

    private final DarienOpModeFSM opMode;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    public AprilTagVisionService(DarienOpModeFSM opMode) {
        this.opMode = opMode;
    }

    public void initializeAprilTagProcessor() {
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = null;
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
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

