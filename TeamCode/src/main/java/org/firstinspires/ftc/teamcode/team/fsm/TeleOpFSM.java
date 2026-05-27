package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.team.config.AutoConfig;
import org.firstinspires.ftc.teamcode.team.config.DriveConfig;
import org.firstinspires.ftc.teamcode.team.core.DriverOneBindings;
import org.firstinspires.ftc.teamcode.team.core.DriverTwoBindings;
import org.firstinspires.ftc.teamcode.team.core.TeleOpIterationResult;
import org.firstinspires.ftc.teamcode.team.core.TeleOpCoordinatorSet;
import org.firstinspires.ftc.teamcode.team.core.TeleOpInitializationCoordinator;
import org.firstinspires.ftc.teamcode.team.core.TeleOpLoopConfig;
import org.firstinspires.ftc.teamcode.team.core.TeleOpLoopContext;
import org.firstinspires.ftc.teamcode.team.core.TeleOpLoopIterationInput;
import org.firstinspires.ftc.teamcode.team.core.TeleOpLoopMetrics;
import org.firstinspires.ftc.teamcode.team.core.TeleOpLoopRuntimeBindings;
import org.firstinspires.ftc.teamcode.team.core.TeleOpStatusCoordinator;

@TeleOp(name = "TeleopFSM", group = "DriverControl")
@Config
@Configurable
public class TeleOpFSM extends DarienOpModeFSM {

    private TeleOpCoordinatorSet coordinators;

    private TeleOpLoopIterationInput buildLoopInput() {
        DriverOneBindings driverOne = coordinators.inputMapper.mapDriverOne(gamepad1);
        DriverTwoBindings driverTwo = coordinators.inputMapper.mapDriverTwo(gamepad2);

        double currentTime = getRuntime();
        double ejectionVelocity = ejectionMotor.getVelocity();
        TeleOpLoopMetrics loopMetrics = TeleOpLoopMetrics.fromShooterMotor(
                currentTime,
                DriveConfig.TICKS_PER_ROTATION,
                ejectionVelocity,
                ejectionMotor.getPower()
        );
        return TeleOpLoopIterationInput.of(driverOne, driverTwo, loopMetrics);
    }

    private TeleOpLoopContext applyLoopResult(TeleOpIterationResult iterationResult) {
        TeleOpLoopContext nextLoopContext = iterationResult.context;
        shootingPowerMode = nextLoopContext.state.shootingPowerMode;

        TeleOpStatusCoordinator.TraceState traceState = iterationResult.traceState;
        addTraceTelemetry("TeleOp", traceState.state, traceState.stateTimerSec);
        telemetry.update();
        return nextLoopContext;
    }

    @Override
    public void initControls() {
        super.initControls();
        gateFSM.close();
        turretFSM.center(); // set to center position
        coordinators = TeleOpCoordinatorSet.create(
                aprilTagService,
                turretFSM,
                intakeFSM,
                shootingFSM,
                gateFSM
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initControls();

        TeleOpInitializationCoordinator.InitializationResult initializationResult = coordinators.initializationCoordinator.initialize(
                preferencesService,
                localizationService,
                coordinators.turretVisionCoordinator,
                telemetry,
                "UNKNOWN",
                AutoConfig.HUMAN_PLAYER_RED_X,
                AutoConfig.HUMAN_PLAYER_RED_Y,
                AutoConfig.HUMAN_PLAYER_BLUE_X,
                AutoConfig.HUMAN_PLAYER_BLUE_Y,
                DriveConfig.ROBOT_CENTER_OFFSET_X,
                DriveConfig.ROBOT_CENTER_OFFSET_Y
        );
        String autoAlliance = initializationResult.autoAlliance;

        waitForStart();
        if (isStopRequested()) return;
        // Start follower in TeleOp drive mode before entering loop.
        follower.startTeleopDrive(true);
        follower.update();

        TeleOpLoopConfig loopConfig = TeleOpLoopConfig.createDefault();
        TeleOpLoopRuntimeBindings loopRuntime = new TeleOpLoopRuntimeBindings(
                localizationService,
                follower,
                intakeFSM,
                shotgunFSM,
                shootingFSM,
                turretFSM,
                gateFSM,
                telemetry
        );
        TeleOpLoopContext loopContext = TeleOpLoopContext.create(
                coordinators,
                loopRuntime,
                autoAlliance,
                shootingPowerMode,
                loopConfig
        );

        while (this.opModeIsActive() && !isStopRequested()) {
            TeleOpLoopIterationInput loopInput = buildLoopInput();

            TeleOpIterationResult iterationResult = coordinators.loopCoordinator.runLoopIteration(
                    loopContext,
                    loopInput
            );
            loopContext = applyLoopResult(iterationResult);
        } //while opModeIsActive

        stopRobot();
    } //runOpMode

} //TeleOpFSM class
