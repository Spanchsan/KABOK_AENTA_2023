package org.firstinspires.ftc.teamcode.drive.opmode.main.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="BochonAga")

public class MainAENTA extends LinearOpMode {
    DcMotor motorFL, motorBL, motorFR, motorBR, liftL, motorLiftL, motorLiftR;
    Servo servoLiftR, servoLiftL, servoKrutilka, claw;
    @Override
    public void runOpMode() throws InterruptedException {
        motorFL = hardwareMap.dcMotor.get("leftF");
        motorBL = hardwareMap.dcMotor.get("leftR");
        motorFR = hardwareMap.dcMotor.get("rightF");
        motorBR = hardwareMap.dcMotor.get("rightR");
        motorLiftL = hardwareMap.dcMotor.get("liftL");
        motorLiftR = hardwareMap.dcMotor.get("liftR");
        servoLiftR = hardwareMap.get(Servo.class, "armR");
        servoLiftL = hardwareMap.get(Servo.class, "armL");
        claw = hardwareMap.get(Servo.class, "claw");
        servoKrutilka = hardwareMap.get(Servo.class, "servoKrutilka");

        servoLiftR.setDirection(Servo.Direction.REVERSE);
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLiftR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        if(opModeIsActive()){
            while (opModeIsActive()){
                double y = -gamepad1.left_stick_y; // Remember, this is reversed!
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x;
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                motorFL.setPower(frontLeftPower);
                motorBL.setPower(backLeftPower);
                motorFR.setPower(frontRightPower);
                motorBR.setPower(backRightPower);

                telemetry.addLine(String.valueOf(frontLeftPower));
                telemetry.addLine(String.valueOf(backLeftPower));
                telemetry.addLine(String.valueOf(frontRightPower));
                telemetry.addLine(String.valueOf(backRightPower));
                telemetry.update();

                if(gamepad1.dpad_up){
                    motorLiftL.setPower(1);
                    motorLiftR.setPower(1);
                } else if(gamepad1.dpad_down){
                    motorLiftL.setPower(-1);
                    motorLiftR.setPower(-1);
                }
                else if(motorLiftL.getCurrentPosition() > 100 && motorLiftR.getCurrentPosition() > 100) {
                    motorLiftL.setPower(0.001);
                    motorLiftR.setPower(0.001);
                }
                else {
                    motorLiftL.setPower(0);
                    motorLiftR.setPower(0);
                }

                if (gamepad1.right_trigger > 0.3) {
                    servoLiftR.setPosition(1);
                    servoLiftL.setPosition(1);
                } else if(gamepad1.left_trigger > 0.3) {
                    servoLiftR.setPosition(0.1);
                    servoLiftL.setPosition(0.1);
                }
                if(gamepad1.x){
                    servoKrutilka.setPosition(0.86);
                }else if(gamepad1.y){
                    servoKrutilka.setPosition(0.2);
                }
                if(gamepad1.left_bumper){
                    claw.setPosition(0.625);
                }else if(gamepad1.right_bumper){
                    claw.setPosition(1);
                }
                telemetry.addLine("motorLift Left: " + motorLiftL.getCurrentPosition());
                telemetry.addLine("motorLift Right: " + motorLiftR.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
