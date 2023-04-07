package org.firstinspires.ftc.teamcode.drive.opmode.main.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.IntakeConstants;

@TeleOp(name = "TestingFunctions")
public class TestingThings extends LinearOpMode {

    DistanceSensor distanceSensor;
    DcMotor motorLiftR, motorLiftL;
    boolean thrWorking = false;
    Servo claw;
    final double CLOSE_INTAKE = IntakeConstants.CLOSE_INTAKE,
            OPEN_INTAKE = IntakeConstants.OPEN_INTAKE,
            rotateGrab = IntakeConstants.ROTATE_GRAB,
            rotatePerevorot = IntakeConstants.ROTATE_PEREVOROT,
            liftGrab = IntakeConstants.LIFT_GRAB,
            liftPerevorot = IntakeConstants.LIFT_PEREVOROT;
    Thread threadHighJ = new Thread(() ->{
        thrWorking = true;
        sleep(100);
        motorLiftL.setTargetPosition(2200);
        motorLiftR.setTargetPosition(2200);
        motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftL.setPower(1);
        motorLiftR.setPower(1);
        while(motorLiftL.isBusy() || motorLiftR.isBusy()) {}
        motorLiftL.setPower(0);
        motorLiftR.setPower(0);
        motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thrWorking = false;
    }),
            threadMidJ = new Thread(() -> {
                thrWorking = true;
                sleep(100);
                motorLiftL.setTargetPosition(1400);
                motorLiftR.setTargetPosition(1400);
                motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLiftL.setPower(1);
                motorLiftR.setPower(1);
                while(motorLiftL.isBusy() || motorLiftR.isBusy()) {}
                motorLiftL.setPower(0);
                motorLiftR.setPower(0);
                motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                thrWorking = false;
            }),
            threadLowJ = new Thread(() -> {
                thrWorking = true;
                sleep(100);
                motorLiftL.setTargetPosition(460);
                motorLiftR.setTargetPosition(460);
                motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLiftL.setPower(1);
                motorLiftR.setPower(1);
                while(motorLiftL.isBusy() || motorLiftR.isBusy()) {}
                motorLiftL.setPower(0);
                motorLiftR.setPower(0);
                motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                thrWorking = false;
            });

    @Override
    public void runOpMode() throws InterruptedException {
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        motorLiftL = hardwareMap.get(DcMotorEx.class, "liftL");
        motorLiftR = hardwareMap.get(DcMotorEx.class,"liftR");
        claw = hardwareMap.get(Servo.class, "claw");
        motorLiftR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        if(opModeIsActive()){
            while(opModeIsActive()){
                if(motorLiftR.getCurrentPosition() < 2860 && motorLiftL.getCurrentPosition() < 2860) {
                    if (gamepad2.dpad_up) {
                        thrWorking = false;
                        motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motorLiftL.setPower(1);
                        motorLiftR.setPower(1);
                    } else if (gamepad2.dpad_down) {
                        thrWorking = false;
                        motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motorLiftL.setPower(-1);
                        motorLiftR.setPower(-1);
                    } else if (motorLiftL.getCurrentPosition() > 100 && motorLiftR.getCurrentPosition() > 100 && !thrWorking) {
                        motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motorLiftL.setPower(0.001);
                        motorLiftR.setPower(0.001);
                    } else if(!thrWorking){
                        motorLiftL.setPower(0);
                        motorLiftR.setPower(0);
                    }
                } else if (gamepad2.dpad_down) {
                    thrWorking = false;
                    motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorLiftL.setPower(-1);
                    motorLiftR.setPower(-1);
                } else {
                    motorLiftL.setPower(0);
                    motorLiftR.setPower(0);
                }
                if(gamepad2.y) {
                    if (!threadHighJ.isAlive())
                        threadHighJ.start();
                } else if(gamepad2.x){
                    if(!threadMidJ.isAlive())
                        threadMidJ.start();
                } else if(gamepad2.a){
                    if(!threadLowJ.isAlive())
                        threadLowJ.start();
                }
                if(gamepad2.left_bumper || gamepad1.left_bumper){
                    telemetry.addLine("LEft BMPER");
                    claw.setPosition(OPEN_INTAKE);
                }else if(gamepad2.right_bumper || gamepad1.right_bumper){
                    telemetry.addLine("RIGHT BMPER");
                    claw.setPosition(CLOSE_INTAKE);
                }
                telemetry.addLine("motorLift Left: " + motorLiftL.getCurrentPosition());
                telemetry.addLine("motorLift Right: " + motorLiftR.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
