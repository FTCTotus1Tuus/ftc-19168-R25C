package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name = "Teleop", group = "DriverControl")
@Config
public class Teleop extends DarienOpModeTeleop {
    boolean isRubberBandsReversed = false;
    // tuning constants for gobilda 117 rpm motor

    // tuning constants for gobilda 312 rpm motor and 4 stage long gobilda viper slide


    public double getIntakeServoPosition() {
        return IntakeServoPosition;
    }
    public void setIntakeServoPosition(double position) {
        IntakeServo.setPosition(position);
        IntakeServoPosition = position;
    }



    @Override
    public void runOpMode() {
        float gain = 2;
        initControls();
        // TODO: Add slowdown for tray init to POS 1 INTAKE
        tp = new TelemetryPacket();
        dash = FtcDashboard.getInstance();
        double startTimeColor = getRuntime();
        waitForStart();
        //Start
        while (this.opModeIsActive()) {
            //pollSensors();
            runDriveSystem();
            NormalizedRGBA colors = intakeColorSensor.getNormalizedColors();
          //assigning the ejectionmotorleft/right controls
            if (gamepad1.y) {
                rubberBands.setPower(INTAKE_RUBBER_BANDS_POWER);
                isRubberBandsReversed = false;
            } else if (gamepad1.a) {
                rubberBands.setPower(OUTPUT_RUBBER_BANDS_POWER);
                isRubberBandsReversed = true;
            } else if (gamepad1.x) {
                rubberBands.setPower(0);
                isRubberBandsReversed = false;
            }

            if (gamepad2.back) {
                shootArtifact();
            }
            if (gamepad2.right_trigger > 0.05 && gamepad2.right_stick_y >= -0.05) {
                ejectionMotorLeft.setPower(getVoltageAdjustedMotorPower(SHOT_GUN_POWER_UP));
                ejectionMotorRight.setPower(-getVoltageAdjustedMotorPower(SHOT_GUN_POWER_UP));
            } else if (gamepad2.right_trigger > 0.05 && gamepad2.right_stick_y < -0.05) {
                ejectionMotorLeft.setPower(getVoltageAdjustedMotorPower(SHOT_GUN_POWER_UP_FAR));
                ejectionMotorRight.setPower(-getVoltageAdjustedMotorPower(SHOT_GUN_POWER_UP_FAR));
            } else {
                ejectionMotorLeft.setPower(0);
                ejectionMotorRight.setPower(0);
            }
            //CONTROL: TRAYINIT
            if (gamepad2.start) {
                setTrayPosition(TRAY_POS_1_INTAKE);
            }
            //CONTROL: EJECTIONMOTOR BACKWARDS
            if (gamepad2.left_trigger > 0.05) {
                ejectionMotorLeft.setPower(-getVoltageAdjustedMotorPower(SHOT_GUN_POWER_DOWN));
                ejectionMotorRight.setPower(getVoltageAdjustedMotorPower(SHOT_GUN_POWER_DOWN));
            }

            //CONTROL: ELEVATOR
            if (gamepad2.left_bumper){
                Elevator.setPosition(ELEVATOR_POS_UP);
            } else {
                Elevator.setPosition(ELEVATOR_POS_DOWN);
            }
            //CONTROL: FEEDER
            if (gamepad2.right_bumper){
                Feeder.setPosition(FEEDER_POS_UP);
            } else {
                Feeder.setPosition(FEEDER_POS_DOWN);
            }
            // CONTROL: INTAKE
            //classify the function
            //when g2.a button is pressed intake servo goes up in increments in relation to the time
            // Update the gain value if either of the A or B gamepad buttons is being held
            if (gamepad1.a) {
                // Only increase the gain by a small amount, since this loop will occur multiple times per second.
                gain += 0.005;
            } else if (gamepad1.b && gain > 1) {
                // A gain of less than 1 will make the values smaller, which is not helpful.
                gain -= 0.005;
            }
            telemetry.addData("Gain", gain);
            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            if (gamepad2.a) {
                IntakeServo.setPosition(INTAKE_SERVO_POS_UP);
                intakeLifted = false; // Cancel any automatic lift
            } else if (intakeColorSensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) intakeColorSensor).getDistance(DistanceUnit.CM));
                if (!intakeLifted && ((DistanceSensor) intakeColorSensor).getDistance(DistanceUnit.CM) <= INTAKE_DISTANCE && (getRuntime() - startTimeColor) >= 1 && !isRubberBandsReversed) {
                    startTimeColor = getRuntime();
                    intakeLifted = true;
                    intakeLiftStartTime = getRuntime();
                }
                if (intakeLifted) {
                    IntakeServo.setPosition(INTAKE_SERVO_POS_UP);
                    if (getRuntime() - intakeLiftStartTime >= INTAKE_SERVO_DURATION_RAISE) {
                        intakeLifted = false;
                    }
                } else {
                    IntakeServo.setPosition(INTAKE_SERVO_POS_DOWN);
                }
            } else {
                IntakeServo.setPosition(INTAKE_SERVO_POS_DOWN);
            }
            telemetry.update();

            /*
            if (isIntakeServoMoving) {
                // Continue moving the servo until the position is reached.
                if (getIntakeServoPosition() < IntakeServoPosUp && getRuntime()-startTimeIntakeServo < 4) {
                    setIntakeServoPosition(getIntakeServoPosition() + 0.005);
                } else {
                    isIntakeServoMoving = false;
                }
            } else {
                if (gamepad2.a) {
                    // Start lifting the intake.
                    startTimeIntakeServo = getRuntime();
                    isIntakeServoMoving = true;
                } else {
                    IntakeServo.setPosition(IntakeServoPosDown);
                }
            }
            */
            // CONTROL: ROTATING TRAY
            if (gamepad2.dpad_left){
                setTrayPosition(TRAY_POS_1_INTAKE);
                //Tray.setPosition(TRAY_POS_1_INTAKE);
                //servoIncremental(TrayServo,TRAY_POS_1_INTAKE,currentTrayPosition, 1,4);
                //currentTrayPosition = TRAY_POS_1_INTAKE;
            } else if (gamepad2.x){
                setTrayPosition(TRAY_POS_1_SCORE);
                //Tray.setPosition(TRAY_POS_1_SCORE);
                //servoIncremental(TrayServo,TRAY_POS_1_SCORE,currentTrayPosition, 1,4);
                //currentTrayPosition = TRAY_POS_1_SCORE;
            } else if (gamepad2.dpad_up){
                setTrayPosition(TRAY_POS_2_INTAKE);
                //Tray.setPosition(TRAY_POS_2_INTAKE);
                //servoIncremental(TrayServo,TRAY_POS_2_INTAKE,currentTrayPosition, 1,4);
                //currentTrayPosition = TRAY_POS_2_INTAKE;
            } else if (gamepad2.y){
                setTrayPosition(TRAY_POS_2_SCORE);
                //Tray.setPosition(TRAY_POS_2_SCORE);
                //servoIncremental(TrayServo,TRAY_POS_2_SCORE,currentTrayPosition, 1,4);
                //currentTrayPosition = TRAY_POS_2_SCORE;
            } else if (gamepad2.dpad_right){
                setTrayPosition(TRAY_POS_3_INTAKE);
                //Tray.setPosition(TRAY_POS_3_INTAKE);
                //servoIncremental(TrayServo,TRAY_POS_3_INTAKE,currentTrayPosition, 1,4);
                //currentTrayPosition = TRAY_POS_3_INTAKE;
            } else if (gamepad2.b){
                setTrayPosition(TRAY_POS_3_SCORE);
                //Tray.setPosition(TRAY_POS_3_SCORE);
                //servoIncremental(TrayServo,TRAY_POS_3_SCORE,currentTrayPosition, 1,4);
                //currentTrayPosition = TRAY_POS_3_SCORE;
            }
            //MACRO: APRILTAG 21
            /*
            tray.setpostion(TRAY_POS_1_SCORE)
            if (gamepad.2(button combo)){
                servoincremental(TRAY_POS_2_SCORE)
                Elevator.setposition(ElevatorUp)
                ejectiomotorleft+right.setpower(1)
                Feeder.setposition(feederup)
                feeder.setposition(down)
                elevation.setposition(down)
                ejectionmotorleft+right.setpower(0)
                servoincermental(trayposition3score)
                Elevator.setposition(ElevatorUp)
                ejectiomotorleft+right.setpower(1)
                Feeder.setposition(feederup)
                feeder.setposition(down)
                elevation.setposition(down)
                ejectionmotorleft+right.setpower(0)
                servoincremental(Traypos1score)
                Elevator.setposition(ElevatorUp)
                ejectiomotorleft+right.setpower(1)
                Feeder.setposition(feederup)
                feeder.setposition(down)
                elevation.setposition(down)
                ejectionmotorleft+right.setpower(0)
            }
             */

        }
    }
}