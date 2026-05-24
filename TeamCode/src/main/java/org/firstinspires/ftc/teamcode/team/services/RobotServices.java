package org.firstinspires.ftc.teamcode.team.services;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;

/**
 * Groups non-hardware robot services used by OpModes and subsystem containers.
 */
public class RobotServices {

    private final DarienOpModeFSM opMode;
    private final AprilTagVisionService visionService;
    private PreferencesService preferencesService;

    private Follower follower;

    public RobotServices(DarienOpModeFSM opMode) {
        this.opMode = opMode;
        this.visionService = new AprilTagVisionService(opMode);
    }

    public void initialize() {
        visionService.initializeAprilTagProcessor();
        preferencesService = new PreferencesService(opMode.hardwareMap.appContext);
        follower = Constants.createFollower(opMode.hardwareMap);
    }

    public Follower getFollower() {
        return follower;
    }

    public AprilTagVisionService getVisionService() {
        return visionService;
    }

    public PreferencesService getPreferencesService() {
        return preferencesService;
    }
}

