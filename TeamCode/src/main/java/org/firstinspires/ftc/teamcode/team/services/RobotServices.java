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
    private LocalizationService localizationService;

    private Follower follower;

    public RobotServices(DarienOpModeFSM opMode) {
        this.opMode = opMode;
        this.visionService = new AprilTagVisionService(opMode);
    }

    public void initialize() {
        visionService.initialize();
        preferencesService = new PreferencesService(opMode.hardwareMap.appContext);
        follower = Constants.createFollower(opMode.hardwareMap);
        localizationService = new LocalizationService(opMode.hardwareMap, follower);
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

    public LocalizationService getLocalizationService() {
        return localizationService;
    }

    /** Tears down camera portal. Call from OpMode stop. */
    public void close() {
        visionService.close();
    }
}

