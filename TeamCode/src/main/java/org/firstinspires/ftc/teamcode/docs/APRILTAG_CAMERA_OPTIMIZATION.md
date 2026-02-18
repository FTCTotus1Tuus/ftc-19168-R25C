# AprilTag Camera Exposure/Gain Optimization Guide

## Problem
The turret auto-align feature doesn't always work from far positions. The camera sometimes produces blurry images and fails to detect AprilTags (IDs 20 and 24) reliably, especially when refreshing the camera view.

## Solution
Implemented manual camera exposure and gain control to eliminate motion blur and improve AprilTag detection at distance.

## What Was Changed

### 1. DarienOpModeFSM.java
- Added camera control imports (`ExposureControl`, `GainControl`, `TimeUnit`)
- Added tunable configuration constants:
  - `APRILTAG_EXPOSURE_MS = 6` (milliseconds, lower = less blur)
  - `APRILTAG_GAIN = 255` (0-255, higher = brighter in low light)
- Added `setManualExposure()` method to configure camera settings
- Added `getCameraSettings()` method to read camera capabilities
- Modified `initAprilTag()` to automatically set manual exposure/gain on initialization

### 2. TuneAprilTagExposure.java (NEW)
Created a dedicated tuning OpMode to experimentally find optimal camera settings for your specific robot and arena lighting.

## How to Use

### Step 1: Initial Testing
1. Deploy the updated code to your robot
2. Run your normal TeleOp or Auto OpMode
3. Test turret auto-align from far positions
4. The camera should now be using exposure=6ms and gain=255 by default

### Step 2: Fine-Tuning (If Needed)
If you still experience detection issues from far positions, tune the values:

1. **Connect a display to view camera feed:**
   - Control Hub: Connect HDMI monitor OR use [scrcpy](https://github.com/Genymobile/scrcpy)
   - Phone RC: View directly on phone screen

2. **Position robot at problematic location:**
   - Place robot at your FARTHEST auto-align position
   - Aim at the goal AprilTag (ID 20 for Blue, ID 24 for Red)

3. **Run the tuning OpMode:**
   - Select "Tune AprilTag Exposure" from Driver Station
   - On Driver Station, tap **3 dots → Camera Stream** to view live feed

4. **Adjust settings using gamepad:**
   - **Left Bumper/Trigger:** Increase/Decrease Exposure
   - **Right Bumper/Trigger:** Increase/Decrease Gain
   - Start with exposure=6ms, gain=max
   - Decrease exposure until image is too dark or tags aren't detected
   - Then increase exposure by 1-2ms for safety margin

5. **Update configuration:**
   - Note the optimal values shown in telemetry
   - Update `DarienOpModeFSM.java`:
     ```java
     public static int APRILTAG_EXPOSURE_MS = [your_value];  // e.g., 6
     public static int APRILTAG_GAIN = [your_value];         // e.g., 255
     ```
   - **OR** use FTC Dashboard to tune in real-time (values are `@Config` annotated)

### Step 3: Real-Time Tuning (Alternative)
If you have FTC Dashboard configured:
1. Open FTC Dashboard in a web browser
2. Navigate to the Configuration tab
3. Find `DarienOpModeFSM` section
4. Adjust `APRILTAG_EXPOSURE_MS` and `APRILTAG_GAIN` while OpMode is running
5. Values take effect on next `initAprilTag()` call (restart OpMode)

## Understanding the Settings

### Exposure (Recommended: 5-6ms)
- **What it does:** Controls how long the camera sensor collects light
- **Lower values (1-5ms):** 
  - ✓ Less motion blur (better for moving robot/turret)
  - ✗ Darker image
- **Higher values (10-50ms):**
  - ✓ Brighter image
  - ✗ More motion blur (poor for detection)

### Gain (Recommended: 200-255)
- **What it does:** Amplifies the signal from the camera sensor
- **Lower values (0-100):**
  - ✓ Less noise/grain
  - ✗ Darker image
- **Higher values (200-255):**
  - ✓ Brighter image in low light
  - ✗ More noise (usually acceptable for AprilTag detection)

### Why This Helps
1. **Default (Auto) Mode Problems:**
   - Camera automatically adjusts exposure based on lighting
   - Can use exposure times of 30-100ms in dim arenas
   - Robot/turret movement causes severe motion blur
   - Blurry tags are impossible to detect

2. **Manual Mode Benefits:**
   - Forces short exposure (5-6ms) → minimal blur
   - High gain compensates for low light
   - Consistent behavior regardless of arena lighting
   - Tags remain sharp even during robot movement

## Testing Checklist

Test auto-align from these positions:
- [ ] Close range (< 3 feet from goal) - should work easily
- [ ] Medium range (3-6 feet) - should still work well  
- [ ] Far range (6+ feet) - this is the critical test
- [ ] Different lighting conditions (bright/dim parts of field)
- [ ] With robot moving vs. stationary
- [ ] After robot has been on for 5+ minutes (thermal effects)

## Troubleshooting

### Tags still not detected from far positions
- **Increase exposure** by 1-2ms at a time (try 7, 8, 9ms)
- **Verify camera focus:** Some webcams have manual focus rings
- **Check camera mounting:** Ensure camera is rigidly mounted (no vibration)
- **Verify camera angle:** Ensure tags are in camera field of view
- **Increase timeout:** Change `TIMEOUT_APRILTAG_DETECTION` from 0.75s to 1.0s or 1.5s

### Image too dark even at max gain
- **Increase exposure** to 8-10ms
- **Check arena lighting:** FTC arenas should have adequate lighting
- **Clean camera lens:** Fingerprints/dust reduce light intake

### Image is blurry
- **Decrease exposure** to 4-5ms (if camera supports it)
- **Reduce robot movement** during detection
- **Check camera mounting** for vibration/flex

### Camera controls not working
- Some cameras don't support manual exposure/gain control
- Check error messages in telemetry
- Try a different webcam (Logitech C920/C930 recommended)

## Advanced: Different Settings for Auto vs. TeleOp

If you want different settings for autonomous vs. teleop:

```java
// In your Auto OpMode:
@Override
public void runOpMode() {
    initControls();
    
    // Override with auto-specific settings
    setManualExposure(5, 255);  // Lower exposure for fast autonomous movements
    
    // ... rest of auto code
}

// In TeleOp:
@Override
public void runOpMode() {
    initControls();
    
    // Override with teleop-specific settings  
    setManualExposure(7, 230);  // Slightly higher exposure for better distance detection
    
    // ... rest of teleop code
}
```

## References
- FTC SDK Sample: `ConceptAprilTagOptimizeExposure.java`
- [AprilTag Detection Values (ftc-docs)](https://ftc-docs.firstinspires.org/apriltag-detection-values)
- Camera: Logitech C910/C920/C930 recommended for manual control support

## Need Help?
1. Check telemetry for error messages during `initAprilTag()`
2. Verify camera is "Webcam 1" in Robot Configuration
3. Test with FTC SDK sample OpModes to verify camera works
4. Check physical camera mounting and lens cleanliness

---
**Implementation Date:** February 12, 2026  
**Team:** FTC 19168  
**Files Modified:** DarienOpModeFSM.java  
**Files Created:** TuneAprilTagExposure.java, APRILTAG_CAMERA_OPTIMIZATION.md

