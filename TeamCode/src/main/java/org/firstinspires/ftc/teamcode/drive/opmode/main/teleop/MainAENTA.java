package org.firstinspires.ftc.teamcode.drive.opmode.main.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.IntakeConstants;
import org.firstinspires.ftc.teamcode.util.Encoder;


@TeleOp(name="BochonAgaSuper")

public class MainAENTA extends LinearOpMode {
    DcMotor motorFL, motorBL, motorFR, motorBR;
    DcMotorEx motorLiftL, motorLiftR;
    Encoder encPerp, encParl;
    Servo servoLiftR, servoLiftL, servoKrutilka, claw,
            servoEnc1, servoEnc2;
    DistanceSensor distanceSensor;

    final double CLOSE_INTAKE = IntakeConstants.CLOSE_INTAKE,
            OPEN_INTAKE = IntakeConstants.OPEN_INTAKE,
        rotateGrab = IntakeConstants.ROTATE_GRAB,
        rotatePerevorot = IntakeConstants.ROTATE_PEREVOROT,
        liftGrab = IntakeConstants.LIFT_GRAB,
        liftPerevorot = IntakeConstants.LIFT_PEREVOROT,
        //қолдың позициясы конусты салудың алдында
        liftIDlE = IntakeConstants.LIFT_IDLE,
        liftThrow = IntakeConstants.LIFT_THROW;
//    Thread threadUP = new Thread(this::intakeUP),
//            threadDOWN = new Thread(this::intakeDOWN);
    Runnable runUP = this::intakeUP,
        runDown = this::intakeDOWN;
    boolean thrWorking = false;
    Runnable runHighJ = () ->{
        thrWorking = true;
        intakeUP();
        sleep(100);
        motorLiftL.setTargetPosition(IntakeConstants.HIGH_JUNC);
        motorLiftR.setTargetPosition(IntakeConstants.HIGH_JUNC);
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
    },
       runMidJ = () -> {
            thrWorking = true;
            intakeUP();
            sleep(100);
            motorLiftL.setTargetPosition(IntakeConstants.MED_JUNC);
            motorLiftR.setTargetPosition(IntakeConstants.MED_JUNC);
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
        },
        runLowJ = () -> {
            thrWorking = true;
            intakeUP();
            sleep(100);
            motorLiftL.setTargetPosition(IntakeConstants.LOW_JUNC);
            motorLiftR.setTargetPosition(IntakeConstants.LOW_JUNC);
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
        };
    Thread threadHighJ, threadMidJ, threadLowJ, threadUp, threadDown;
//    Thread threadHighJ = new Thread(() ->{
//        thrWorking = true;
//        intakeUP();
//        sleep(100);
//        motorLiftL.setTargetPosition(IntakeConstants.HIGH_JUNC);
//        motorLiftR.setTargetPosition(IntakeConstants.HIGH_JUNC);
//        motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorLiftL.setPower(1);
//        motorLiftR.setPower(1);
//        while(motorLiftL.isBusy() || motorLiftR.isBusy()) {}
//        motorLiftL.setPower(0);
//        motorLiftR.setPower(0);
//        motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        thrWorking = false;
//    }),
//    threadMidJ = new Thread(() -> {
//        thrWorking = true;
//        intakeUP();
//        sleep(100);
//        motorLiftL.setTargetPosition(IntakeConstants.MED_JUNC);
//        motorLiftR.setTargetPosition(IntakeConstants.MED_JUNC);
//        motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorLiftL.setPower(1);
//        motorLiftR.setPower(1);
//        while(motorLiftL.isBusy() || motorLiftR.isBusy()) {}
//        motorLiftL.setPower(0);
//        motorLiftR.setPower(0);
//        motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        thrWorking = false;
//    }),
//    threadLowJ = new Thread(() -> {
//        thrWorking = true;
//        intakeUP();
//        sleep(100);
//        motorLiftL.setTargetPosition(IntakeConstants.LOW_JUNC);
//        motorLiftR.setTargetPosition(IntakeConstants.LOW_JUNC);
//        motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motorLiftL.setPower(1);
//        motorLiftR.setPower(1);
//        while(motorLiftL.isBusy() || motorLiftR.isBusy()) {}
//        motorLiftL.setPower(0);
//        motorLiftR.setPower(0);
//        motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        thrWorking = false;
//    });


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        if(opModeIsActive()){
            initStart();
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
                if(gamepad1.dpad_up){
                    motorFL.setPower(0.5);
                    motorBL.setPower(0.5);
                    motorFR.setPower(0.5);
                    motorBR.setPower(0.5);
                } else if(gamepad1.dpad_down){
                    motorFL.setPower(-0.5);
                    motorBL.setPower(-0.5);
                    motorFR.setPower(-0.5);
                    motorBR.setPower(-0.5);
                }
                telemetry.addLine(String.valueOf(frontLeftPower));
                telemetry.addLine(String.valueOf(backLeftPower));
                telemetry.addLine(String.valueOf(frontRightPower));
                telemetry.addLine(String.valueOf(backRightPower));
                //2860 max height lift
                //2220 high junction
                //
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
                        motorLiftL.setPower(0.0025);
                        motorLiftR.setPower(0.0025);
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

                if(!thrWorking && distanceSensor.getDistance(DistanceUnit.CM) < 2.5 && motorLiftL.getCurrentPosition() != 0 && motorLiftR.getCurrentPosition() != 0){
                    motorLiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorLiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                }
                if(gamepad2.y) {
//                    if (!threadHighJ.isAlive())
//                        threadHighJ.start();
                    if(threadHighJ == null || !threadHighJ.isAlive()){
                        threadHighJ = new Thread(runHighJ);
                        threadHighJ.start();
                    }
                } else if(gamepad2.x){
//                    if(!threadMidJ.isAlive())
//                        threadMidJ.start();
                    if(threadMidJ == null || !threadMidJ.isAlive()){
                        threadMidJ = new Thread(runMidJ);
                        threadMidJ.start();
                    }
                } else if(gamepad2.a){
//                    if(!threadLowJ.isAlive())
//                        threadLowJ.start();
                    if(threadLowJ == null || !threadLowJ.isAlive()){
                        threadLowJ = new Thread(runLowJ);
                        threadLowJ.start();
                    }
                }
                if(gamepad1.x){
                    servoKrutilka.setPosition(rotateGrab);
                }else if(gamepad1.y){
                    servoKrutilka.setPosition(rotatePerevorot);
                }
                if (gamepad2.right_trigger > 0.3) {
                    //PEREVOROT
                    setServPosLift(liftPerevorot);
                } else if(gamepad2.left_trigger > 0.3) {
                    //GRAB
                    setServPosLift(liftThrow);
                }
                if(gamepad2.right_stick_button){
//                    if(!threadUP.isAlive())
//                        threadUP.start();
                    if(threadUp == null || !threadUp.isAlive()){
                        threadUp = new Thread(runUP);
                        threadUp.start();
                    }
                } else if(gamepad2.b){
//                    if(!threadDOWN.isAlive())
//                        threadDOWN.start();
                    if(threadDown == null || !threadDown.isAlive()){
                        threadDown = new Thread(runDown);
                        threadDown.start();
                    }
                }
                if(gamepad2.left_bumper || gamepad1.left_bumper){
                    telemetry.addLine("LEft BMPER");
                    claw.setPosition(OPEN_INTAKE);
                }else if(gamepad2.right_bumper || gamepad1.right_bumper){
                    telemetry.addLine("RIGHT BMPER");
                    claw.setPosition(CLOSE_INTAKE);
                }
                if(gamepad1.b){
                    motorFR.setPower(1);
                }else if(gamepad1.a){
                    motorBR.setPower(1);
                }else if(gamepad1.y){
                    motorFL.setPower(1);
                }else if(gamepad1.x){
                    motorBL.setPower(1);
                }
                telemetry.addLine("motorLift Left: " + motorLiftL.getCurrentPosition());
                telemetry.addLine("motorLift Right: " + motorLiftR.getCurrentPosition());
                telemetry.addLine("Encoder Parl pos: " + encParl.getCurrentPosition());
                telemetry.addLine("Encoder Perp pos: " + encPerp.getCurrentPosition());
                telemetry.addLine("Distance Sensor: " + distanceSensor.getDistance(DistanceUnit.CM));
                telemetry.addLine("LEFT FRONT: " + motorFL.getCurrentPosition());
                telemetry.addLine("LEFT REAR: " + motorBL.getCurrentPosition());
                telemetry.addLine("RIGHT FRONT" + motorFR.getCurrentPosition());
                telemetry.addLine("RIGHT REAR: " + motorBR.getCurrentPosition());
                telemetry.update();
            }
        }
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
        sleep(250);
        servoKrutilka.setPosition(rotatePerevorot);
        sleep(300);
        setServPosLift(liftThrow);
    }

    /**
     * ARM-ды конусты алудың позициясына қояды
     */
    private void intakeDOWN(){
        if(servoLiftL.getPosition() > liftIDlE) {
            claw.setPosition(0.825);
            setServPosLift(liftIDlE - 0.1);
            sleep(400);
            servoKrutilka.setPosition(rotateGrab);
            sleep(300);
            setServPosLift(liftGrab);
            sleep(150);
            claw.setPosition(OPEN_INTAKE);
        } else {
            setServPosLift(liftIDlE - 0.1);
            sleep(100);
            servoKrutilka.setPosition(rotateGrab);
            sleep(250);
            setServPosLift(liftGrab);
            claw.setPosition(OPEN_INTAKE);
        }
    }

    private void initHardware(){
        motorFL = hardwareMap.dcMotor.get("leftF");
        motorBL = hardwareMap.dcMotor.get("leftR");
        motorFR = hardwareMap.dcMotor.get("rightF");
        motorBR = hardwareMap.dcMotor.get("rightR");
        motorLiftL = hardwareMap.get(DcMotorEx.class, "liftL");
        motorLiftR = hardwareMap.get(DcMotorEx.class,"liftR");
        servoLiftR = hardwareMap.get(Servo.class, "armR");
        servoLiftL = hardwareMap.get(Servo.class, "armL");
        claw = hardwareMap.get(Servo.class, "claw");
        servoKrutilka = hardwareMap.get(Servo.class, "servoKrutilka");
        servoEnc1 = hardwareMap.get(Servo.class, "servoEnc1");
        servoEnc2 = hardwareMap.get(Servo.class, "servoEnc2");

        encPerp = new Encoder(hardwareMap.get(DcMotorEx.class, "encPerp"));
        encParl = new Encoder(hardwareMap.get(DcMotorEx.class, "encParl"));

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        servoLiftR.setDirection(Servo.Direction.REVERSE);
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLiftR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLiftR.setTargetPositionTolerance(70);
        motorLiftL.setTargetPositionTolerance(70);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
    private void initStart(){
        if(threadDown == null || !threadDown.isAlive()){
            threadDown = new Thread(runDown);
            threadDown.start();
        }
        servoEnc1.setPosition(0);
        servoEnc2.setPosition(1);
    }
}
