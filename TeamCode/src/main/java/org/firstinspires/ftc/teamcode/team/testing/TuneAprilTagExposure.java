package org.firstinspires.ftc.teamcode.team.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * This OpMode helps tune camera Exposure and Gain for optimal AprilTag detection.
 * <p>
 * GOAL: Find the lowest (shortest) Exposure value that still provides reliable Tag Detection
 * from your farthest shooting positions.
 * <p>
 * INSTRUCTIONS:
 * 1. Connect an HDMI monitor to Control Hub OR use scrcpy to view the camera stream
 * 2. Position robot at your FAR shooting position where AprilTags are hard to detect
 * 3. Start this OpMode and view the camera preview (3 dots > Camera Stream on Driver Station)
 * 4. Use Left bumper/trigger to adjust Exposure (start low, increase until tags detected)
 * 5. Use Right bumper/trigger to adjust Gain (start high for better low-light performance)
 * 6. Note the optimal values and update APRILTAG_EXPOSURE_MS and APRILTAG_GAIN in DarienOpModeFSM
 * <p>
 * TIPS:
 * - Lower exposure = less motion blur but darker image
 * - Higher gain = brighter image but more noise
 * - Start with exposure 5-6ms and gain at maximum, then adjust
 * - Test from your farthest turret auto-align positions
 */
@Config
@TeleOp(name = "Tune AprilTag Exposure", group = "Testing")
public class TuneAprilTagExposure extends LinearOpMode {

    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTag;

    private int myExposure;
    private int minExposure;
    private int maxExposure;
    private int myGain;
    private int minGain;
    private int maxGain;

    // Track button states for edge detection
    boolean thisExpUp = false;
    boolean thisExpDn = false;
    boolean thisGainUp = false;
    boolean thisGainDn = false;
    boolean lastExpUp = false;
    boolean lastExpDn = false;
    boolean lastGainUp = false;
    boolean lastGainDn = false;

    @Override
    public void runOpMode() {
        // Initialize the AprilTag Detection process
        initAprilTag();

        // Get camera min/max settings, then set low exposure with high gain
        getCameraSettings();
        myExposure = Math.min(6, minExposure);  // Start at 6ms or minimum
        myGain = maxGain;  // Start at maximum gain
        setManualExposure(myExposure, myGain);

        // Wait for start
        telemetry.addLine("=== APRILTAG EXPOSURE/GAIN TUNING ===");
        telemetry.addLine();
        telemetry.addLine("CAMERA PREVIEW:");
        telemetry.addData("  Enable on Driver Station", "3 dots > Camera Stream");
        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addData("  Left Bumper", "Increase Exposure");
        telemetry.addData("  Left Trigger", "Decrease Exposure");
        telemetry.addData("  Right Bumper", "Increase Gain");
        telemetry.addData("  Right Trigger", "Decrease Gain");
        telemetry.addLine();
        telemetry.addLine("GOAL: Find LOWEST exposure with reliable detection");
        telemetry.addLine();
        telemetry.addData(">", "Touch START to begin tuning");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Display instructions
            telemetry.addLine("=== TUNING APRILTAG CAMERA ===");
            telemetry.addLine();
            telemetry.addLine("Find LOWEST Exposure with reliable detection.");
            telemetry.addLine("Left bump/trig = Exposure | Right bump/trig = Gain");
            telemetry.addLine();

            // Display detection status
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            int numTags = currentDetections.size();
            if (numTags > 0) {
                telemetry.addData("Status", "✓ %d TAG(S) DETECTED ✓", numTags);
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        telemetry.addData("  Tag ID %d", detection.id);
                        telemetry.addData("    Range", "%.1f inches", detection.ftcPose.range);
                    }
                }
            } else {
                telemetry.addData("Status", "✗ NO TAGS DETECTED ✗");
            }
            telemetry.addLine();

            // Display current settings
            telemetry.addData("Exposure (ms)", "%d  (range: %d - %d)", myExposure, minExposure, maxExposure);
            telemetry.addData("Gain", "%d  (range: %d - %d)", myGain, minGain, maxGain);
            telemetry.addLine();

            // Display recommendations
            if (numTags > 0 && myExposure > 5) {
                telemetry.addLine("TIP: Tags detected! Try DECREASING exposure");
            } else if (numTags == 0 && myExposure < maxExposure) {
                telemetry.addLine("TIP: No tags. Try INCREASING exposure or gain");
            } else if (numTags > 0) {
                telemetry.addLine("✓ GOOD! Update DarienOpModeFSM with these values:");
                telemetry.addData("  APRILTAG_EXPOSURE_MS", myExposure);
                telemetry.addData("  APRILTAG_GAIN", myGain);
            }

            telemetry.update();

            // Read gamepad buttons
            thisExpUp = gamepad1.left_bumper;
            thisExpDn = gamepad1.left_trigger > 0.25;
            thisGainUp = gamepad1.right_bumper;
            thisGainDn = gamepad1.right_trigger > 0.25;

            // Adjust exposure on button press (edge detection)
            if (thisExpUp && !lastExpUp) {
                myExposure = Range.clip(myExposure + 1, minExposure, maxExposure);
                setManualExposure(myExposure, myGain);
            } else if (thisExpDn && !lastExpDn) {
                myExposure = Range.clip(myExposure - 1, minExposure, maxExposure);
                setManualExposure(myExposure, myGain);
            }

            // Adjust gain on button press
            if (thisGainUp && !lastGainUp) {
                myGain = Range.clip(myGain + 5, minGain, maxGain);  // Increment by 5 for faster adjustment
                setManualExposure(myExposure, myGain);
            } else if (thisGainDn && !lastGainDn) {
                myGain = Range.clip(myGain - 5, minGain, maxGain);
                setManualExposure(myExposure, myGain);
            }

            // Save button states
            lastExpUp = thisExpUp;
            lastExpDn = thisExpDn;
            lastGainUp = thisGainUp;
            lastGainDn = thisGainDn;

            sleep(20);
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal for Webcam 1
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Manually set the camera gain and exposure.
     * Can only be called AFTER calling initAprilTag().
     *
     * @param exposureMS Exposure time in milliseconds
     * @param gain       Gain value
     * @return true if controls are set
     */
    private boolean setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return false;
        }

        // Wait for camera to be streaming
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting...");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls
        if (!isStopRequested()) {
            try {
                // Set exposure to Manual mode
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
                sleep(20);

                // Set gain
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(gain);
                sleep(20);

                return true;
            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
                telemetry.update();
                return false;
            }
        }
        return false;
    }

    /**
     * Read this camera's minimum and maximum Exposure and Gain settings.
     * Can only be called AFTER calling initAprilTag().
     */
    private void getCameraSettings() {
        if (visionPortal == null) {
            return;
        }

        // Wait for camera to be streaming
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting...");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Get camera limits
        if (!isStopRequested()) {
            try {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
                maxExposure = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                minGain = gainControl.getMinGain();
                maxGain = gainControl.getMaxGain();
            } catch (Exception e) {
                telemetry.addData("Error reading camera settings", e.getMessage());
                telemetry.update();
                // Set defaults if reading fails
                minExposure = 1;
                maxExposure = 100;
                minGain = 0;
                maxGain = 255;
            }
        }
    }
}

