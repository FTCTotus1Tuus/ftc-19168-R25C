package org.firstinspires.ftc.teamcode.team.core;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.GateFSM;
import org.firstinspires.ftc.teamcode.team.fsm.IntakeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootingFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShotgunFSM;
import org.firstinspires.ftc.teamcode.team.fsm.TurretFSM;

/**
 * Coordinates TeleOp auto-park state progression and path start logic.
 */
public class AutoParkCoordinator {

    public static class AutoParkResult {
        public final boolean isAutoParking;
        public final double autoParkStartTime;
        public final DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch;

        public AutoParkResult(boolean isAutoParking, double autoParkStartTime, DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch) {
            this.isAutoParking = isAutoParking;
            this.autoParkStartTime = autoParkStartTime;
            this.shotgunPowerLatch = shotgunPowerLatch;
        }
    }

    public AutoParkResult updateAutoParkProgress(
            boolean isAutoParking,
            double autoParkStartTime,
            double currentTime,
            double leftStickX,
            double leftStickY,
            double rightStickX,
            double stickDeadzone,
            double autoParkTimeout,
            Follower follower,
            Telemetry telemetry,
            DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch
    ) {
        if (!isAutoParking) {
            return new AutoParkResult(false, autoParkStartTime, shotgunPowerLatch);
        }

        boolean driverStickInput =
                Math.abs(leftStickX) > stickDeadzone ||
                Math.abs(leftStickY) > stickDeadzone ||
                Math.abs(rightStickX) > stickDeadzone;
        boolean timedOut = (currentTime - autoParkStartTime) > autoParkTimeout;

        if (driverStickInput) {
            follower.startTeleopDrive(true);
            telemetry.addLine("AUTO-PARK: Cancelled by driver!");
            return new AutoParkResult(false, autoParkStartTime, shotgunPowerLatch);
        }

        if (!follower.isBusy() || timedOut) {
            follower.startTeleopDrive(true);
            telemetry.addLine("AUTO-PARK: Complete!");
            return new AutoParkResult(false, autoParkStartTime, shotgunPowerLatch);
        }

        return new AutoParkResult(true, autoParkStartTime, shotgunPowerLatch);
    }

    public AutoParkResult tryStartAutoPark(
            boolean autoParkRequested,
            boolean isAutoParking,
            double currentTime,
            double autoParkStartTime,
            String autoAlliance,
            double parkRedX,
            double parkRedY,
            double parkRedHeadingDeg,
            double parkBlueX,
            double parkBlueY,
            double parkBlueHeadingDeg,
            double autoParkPower,
            Follower follower,
            IntakeFSM intakeFSM,
            ShotgunFSM shotgunFSM,
            ShootingFSM shootingFSM,
            TurretFSM turretFSM,
            GateFSM gateFSM,
            DarienOpModeFSM.ShotgunPowerLevel shotgunPowerLatch
    ) {
        if (!autoParkRequested || isAutoParking) {
            return new AutoParkResult(isAutoParking, autoParkStartTime, shotgunPowerLatch);
        }

        double parkX;
        double parkY;
        double parkHeadingDeg;
        if ("RED".equals(autoAlliance)) {
            parkX = parkRedX;
            parkY = parkRedY;
            parkHeadingDeg = parkRedHeadingDeg;
        } else {
            parkX = parkBlueX;
            parkY = parkBlueY;
            parkHeadingDeg = parkBlueHeadingDeg;
        }

        Pose currentPose = follower.getPose();
        PathChain parkPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(currentPose.getX(), currentPose.getY()),
                        new Pose(parkX, parkY)
                ))
                .setLinearHeadingInterpolation(currentPose.getHeading(), Math.toRadians(parkHeadingDeg))
                .build();

        intakeFSM.off();
        shotgunPowerLatch = DarienOpModeFSM.ShotgunPowerLevel.OFF;
        shotgunFSM.toOff();
        shootingFSM.reset();
        turretFSM.center();
        turretFSM.setState(TurretFSM.TurretStates.MANUAL);
        gateFSM.close();

        follower.setMaxPower(autoParkPower);
        follower.followPath(parkPath, true);

        return new AutoParkResult(true, currentTime, shotgunPowerLatch);
    }
}


