package org.firstinspires.ftc.teamcode.team.fsm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "TeleopFSM2", group = "DriverControl")
@Config
@Configurable
public class TeleOpFSM2 extends DarienOpModeFSM {

    // INSTANCES
    public TelemetryManager panelsTelemetry;   // Panels Telemetry instance
    // follower is inherited from DarienOpModeFSM
    private GoBildaPinpointDriver odo;          // Pinpoint odometry driver for position reset

    // TUNING CONSTANTS
    public static double ROTATION_SCALE = 0.5;
    public static double SPEED_SCALE = 1.0;
    public static double SPEED_SCALE_TURN = 0.8;
    public static double INPUT_EXPONENT = 3.0; // 1.0=linear, 2.0=squared, 3.0=cubed (preserves sign)

    // VARIABLES
    double directionX = 0;
    double directionY = 0;


    @Override
    public void initControls() {
        super.initControls();

        // Initialize GoBildaPinpointDriver for odometry position reset
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    }


    @Override
    public void runOpMode() throws InterruptedException {
        float gain = 2;
        initControls();
        tp = new TelemetryPacket();
        dash = FtcDashboard.getInstance();


        waitForStart();
        if (isStopRequested()) return;
        //Start
        follower.startTeleopDrive(true);
        follower.update();

        while (this.opModeIsActive() && !isStopRequested()) {

            // -----------------
            // ALWAYS RUN
            // -----------------
            runDriveSystem();

            telemetry.addData("L  x", gamepad1.left_stick_x);
            telemetry.addData("L  y", gamepad1.left_stick_y);
            telemetry.addData("R  x", gamepad1.right_stick_x);
            telemetry.addData("R  y", gamepad1.right_stick_y);


            //panelsTelemetry.update(telemetry);
            telemetry.update();
        } //while opModeIsActive
    } //runOpMode


    public void runDriveSystem() {
        directionX = Math.pow(gamepad1.left_stick_x, 5);
        directionY = Math.pow(-gamepad1.left_stick_y, 5);
        double rotation = Math.pow(gamepad1.right_stick_x, 5);
        MoveRobot(directionX, directionY, rotation);
    }

    public void MoveRobot(double x, double y, double rotation) {

        //based on vectors, front pointing in, back pointing out, and all wheels set to FORWARD in config

        double divBy = 1;
        double wheel0 = clamp(x + y + rotation, -1, 1);
        double wheel1 = clamp(-x + y - rotation, -1, 1);
        double wheel2 = clamp(-x + y + rotation, -1, 1);
        double wheel3 = clamp(x + y - rotation, -1, 1);

        //divBy = (gamepad1.left_trigger / 2) + 0.5;
        //telemetry.addData("", wheel0 * divBy);

        MoveMotor(omniMotor0, wheel0 * divBy);
        MoveMotor(omniMotor1, wheel1 * divBy);
        MoveMotor(omniMotor2, wheel2 * divBy);
        MoveMotor(omniMotor3, wheel3 * divBy);
    }

    public void MoveMotor(DcMotor motor, double power) {
         /*This function just moves the motors and updates the
         logs for replay*/
        motor.setPower(power);
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }


} //TeleOpFSM class
