package org.firstinspires.ftc.teamcode.team.core;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.team.MotorHelper;
import org.firstinspires.ftc.teamcode.team.fsm.AprilTagDetectionFSM;
import org.firstinspires.ftc.teamcode.team.fsm.DarienOpModeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.GateFSM;
import org.firstinspires.ftc.teamcode.team.fsm.IntakeFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootArtifactFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootPatternFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShootingFSM;
import org.firstinspires.ftc.teamcode.team.fsm.ShotgunFSM;
import org.firstinspires.ftc.teamcode.team.fsm.TurretFSM;
import org.firstinspires.ftc.teamcode.team.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.team.services.AprilTagVisionService;
import org.firstinspires.ftc.teamcode.team.services.PreferencesService;
import org.firstinspires.ftc.teamcode.team.services.RobotServices;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Composes robot hardware, services, and subsystem FSM instances for an OpMode.
 */
public class RobotContainer {

    private final DarienOpModeFSM opMode;
    private final RobotHardware hardware;
    private final RobotServices services;

    private MotorHelper motorHelper;

    private AprilTagDetectionFSM tagFSM;
    private ShootPatternFSM shootPatternFSM;
    private ShootArtifactFSM shootArtifactFSM;
    private ShotgunFSM shotgunFSM;
    private TurretFSM turretFSM;
    private GateFSM gateFSM;
    private IntakeFSM intakeFSM;
    private ShootingFSM shootingFSM;

    public RobotContainer(DarienOpModeFSM opMode) {
        this.opMode = opMode;
        this.hardware = new RobotHardware();
        this.services = new RobotServices(opMode);
    }

    public void initialize() {
        hardware.initialize(opMode.hardwareMap);
        services.initialize();

        motorHelper = new MotorHelper(opMode.telemetry, DarienOpModeFSM.TICKS_PER_ROTATION);

        tagFSM = new AprilTagDetectionFSM(services.getVisionService().getAprilTagProcessor(), DarienOpModeFSM.TIMEOUT_APRILTAG_DETECTION);
        shootArtifactFSM = new ShootArtifactFSM(opMode);
        shootPatternFSM = new ShootPatternFSM(opMode);
        shotgunFSM = new ShotgunFSM(
                DarienOpModeFSM.SHOT_GUN_POWER_UP,
                DarienOpModeFSM.SHOT_GUN_POWER_UP_FAR,
                hardware.ejectionMotor,
                opMode,
                motorHelper
        );

        turretFSM = new TurretFSM(opMode.hardwareMap);
        turretFSM.init();

        gateFSM = new GateFSM(opMode.hardwareMap);
        gateFSM.init();

        intakeFSM = new IntakeFSM(opMode.hardwareMap, gateFSM);
        intakeFSM.init();

        shootingFSM = new ShootingFSM(gateFSM, shotgunFSM, intakeFSM, opMode);
    }

    public RobotHardware getHardware() {
        return hardware;
    }

    public Follower getFollower() {
        return services.getFollower();
    }

    public MotorHelper getMotorHelper() {
        return motorHelper;
    }

    public AprilTagProcessor getAprilTag() {
        return services.getVisionService().getAprilTagProcessor();
    }

    public VisionPortal getVisionPortal() {
        return services.getVisionService().getVisionPortal();
    }

    public AprilTagVisionService getVisionService() {
        return services.getVisionService();
    }

    public PreferencesService getPreferencesService() {
        return services.getPreferencesService();
    }

    public AprilTagDetectionFSM getTagFSM() {
        return tagFSM;
    }

    public ShootPatternFSM getShootPatternFSM() {
        return shootPatternFSM;
    }

    public ShootArtifactFSM getShootArtifactFSM() {
        return shootArtifactFSM;
    }

    public ShotgunFSM getShotgunFSM() {
        return shotgunFSM;
    }

    public TurretFSM getTurretFSM() {
        return turretFSM;
    }

    public GateFSM getGateFSM() {
        return gateFSM;
    }

    public IntakeFSM getIntakeFSM() {
        return intakeFSM;
    }

    public ShootingFSM getShootingFSM() {
        return shootingFSM;
    }
}

