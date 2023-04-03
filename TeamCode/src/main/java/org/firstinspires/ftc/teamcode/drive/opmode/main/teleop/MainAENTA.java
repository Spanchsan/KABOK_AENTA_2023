package org.firstinspires.ftc.teamcode.drive.opmode.main.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.IntakeConstants;
import org.firstinspires.ftc.teamcode.util.Encoder;


@TeleOp(name="BochonAgaSuper")

public class MainAENTA extends LinearOpMode {
    DcMotor motorFL, motorBL, motorFR, motorBR, liftL, motorLiftL, motorLiftR;
    //Encoder encoder;
    Servo servoLiftR, servoLiftL, servoKrutilka, claw;
    final double CLOSE_INTAKE = IntakeConstants.CLOSE_INTAKE,
            OPEN_INTAKE = IntakeConstants.OPEN_INTAKE,
        rotateGrab = IntakeConstants.ROTATE_GRAB,
        rotatePerevorot = IntakeConstants.ROTATE_PEREVOROT,
        liftGrab = IntakeConstants.LIFT_GRAB,
        liftPerevorot = IntakeConstants.LIFT_PEREVOROT,
        //қолдың позициясы конусты салудың алдында
        liftIDlE = IntakeConstants.LIFT_IDLE;
    Thread threadUP = new Thread(this::intakeUP),
            threadDOWN = new Thread(this::intakeDOWN);

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
        //encoder = new Encoder(hardwareMap.get(DcMotorEx.class, "enc1"));

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
                double rx = gamepad1.right_stick_x * 0.8;
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
                if(gamepad2.dpad_up){
                    motorLiftL.setPower(1);
                    motorLiftR.setPower(1);
                } else if(gamepad2.dpad_down){
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
                if(gamepad2.x){
                    servoKrutilka.setPosition(rotateGrab);
                }else if(gamepad2.y){
                    servoKrutilka.setPosition(rotatePerevorot);
                }
                if (gamepad2.right_trigger > 0.3) {
                    //PEREVOROT
                    setServPosLift(liftPerevorot);
                } else if(gamepad2.left_trigger > 0.3) {
                    //GRAB
                    setServPosLift(liftGrab);
                }
                if(gamepad2.a){
                    if(!threadUP.isAlive())
                        threadUP.start();
                } else if(gamepad2.b){
                    if(!threadDOWN.isAlive())
                        threadDOWN.start();
                }
                if(gamepad2.left_bumper || gamepad1.left_bumper){
                    claw.setPosition(OPEN_INTAKE);
                }else if(gamepad2.right_bumper || gamepad1.right_bumper){
                    claw.setPosition(CLOSE_INTAKE);
                }
                telemetry.addLine("motorLift Left: " + motorLiftL.getCurrentPosition());
                telemetry.addLine("motorLift Right: " + motorLiftR.getCurrentPosition());
               // telemetry.addLine("Encoder pos: " + encoder.getCurrentPosition());
                telemetry.update();
            }
        }
    }

    /**
     * @param pos Position of the Servo
     */
    private void setServPosLift(double pos){
        servoLiftR.setPosition(pos);
        servoLiftL.setPosition(pos);
    }

    /**
     * Ставит ARM в Стандартное положение(Хватать конус)
     *
     * БОЧН ЧОРТ
     */
    private void initPOS(){
        claw.setPosition(OPEN_INTAKE);
        servoKrutilka.setPosition(rotateGrab);
        setServPosLift(liftGrab);
    }

    /**
     * Ставит ARM и переворачивает CLAW конустың салудың алдында
     *
     * КАБК черт
     */
    private void intakeUP(){
        //initPOS();
        //sleep(100);
        claw.setPosition(CLOSE_INTAKE);
        sleep(100);
        setServPosLift(0.35);
        sleep(150);
        servoKrutilka.setPosition(rotatePerevorot);
        sleep(400);
        setServPosLift(liftIDlE);
    }

    /**
     * ARM-ды конусты алудың позициясына қояды
     */
    private void intakeDOWN(){
        if(servoLiftL.getPosition() > liftIDlE) {
            claw.setPosition(0.8);
            setServPosLift(liftIDlE - 0.1);
            sleep(400);
            servoKrutilka.setPosition(rotateGrab);
            sleep(300);
            setServPosLift(liftGrab + 0.05);
            sleep(150);
            claw.setPosition(OPEN_INTAKE);
        } else {
            setServPosLift(liftIDlE - 0.1);
            sleep(100);
            servoKrutilka.setPosition(rotateGrab);
            sleep(250);
            setServPosLift(liftGrab + 0.05);
            claw.setPosition(OPEN_INTAKE);
        }
    }
}
