package org.firstinspires.ftc.teamcode.team;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Config
public class ImageProcess implements VisionProcessor {
    private Mat workingMat1 = new Mat(), workingMat2 = new Mat(), workingMatRed = new Mat(), workingMatYellow = new Mat();
    public static int minHueR = 0, minSaturationR = 40, minValueR = 0, maxHueR = 10, maxSaturationR = 260, maxValueR = 300,
            minHueY = 15, minSaturationY = 245, minValueY = 110, maxHueY = 35, maxSaturationY = 255,
            maxValueY = 255, frameWidth, frameHeight;

    private double redCount, yellowCount;
    private boolean lastResult;

    public boolean getLastResult() {
        return lastResult;
    }


    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        workingMat1 = frame.clone();

        Imgproc.cvtColor(workingMat1, workingMat2, Imgproc.COLOR_RGB2HSV);
        Core.inRange(workingMat2, new Scalar(minHueR, minSaturationR, minValueR), new Scalar(maxHueR, maxSaturationR, maxValueR), workingMatRed);
        Core.inRange(workingMat2, new Scalar(minHueY, minSaturationY, minValueY), new Scalar(maxHueY, maxSaturationY, maxValueY), workingMatYellow);

        redCount = Core.countNonZero(workingMatRed);
        yellowCount = Core.countNonZero(workingMatYellow);

        lastResult = redCount > yellowCount;

        workingMat1.release();
        workingMat2.release();
        workingMatRed.release();
        workingMatYellow.release();

        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
