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
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name = "TestingFunctions")
public class TestingThings extends LinearOpMode {

    DcMotor motorFL, motorBL, motorFR, motorBR;
    DcMotorEx motorLiftL, motorLiftR;
    Encoder encPerp, encParl;
    Servo servoLiftR, servoLiftL, servoKrutilka, claw,
            servoEnc1, servoEnc2;
    final double CLOSE_INTAKE = IntakeConstants.CLOSE_INTAKE,
            OPEN_INTAKE = IntakeConstants.OPEN_INTAKE,
            rotateGrab = IntakeConstants.ROTATE_GRAB,
            rotatePerevorot = IntakeConstants.ROTATE_PEREVOROT,
            liftGrab = IntakeConstants.LIFT_GRAB,
            liftPerevorot = IntakeConstants.LIFT_PEREVOROT,
    //қолдың позициясы конусты салудың алдында
    liftIDlE = IntakeConstants.LIFT_IDLE,
            liftThrow = IntakeConstants.LIFT_THROW;
    Runnable runUP = this::intakeUP,
            runDown = this::intakeDOWN;
    boolean thrWorking = false;
    Runnable runHighJ = () ->{
        //thrWorking = true;
        new Thread(this::intakeUP).start();
        sleep(100);
        motorLiftL.setTargetPosition(IntakeConstants.HIGH_JUNC);
        motorLiftR.setTargetPosition(IntakeConstants.HIGH_JUNC);
        motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLiftL.setPower(1);
        motorLiftR.setPower(1);

    },
            runMidJ = () -> {
                //thrWorking = true;
                new Thread(this::intakeUP).start();
                sleep(100);
                motorLiftL.setTargetPosition(IntakeConstants.MED_JUNC);
                motorLiftR.setTargetPosition(IntakeConstants.MED_JUNC);
                motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLiftL.setPower(1);
                motorLiftR.setPower(1);

            },
            runLowJ = () -> {
                //thrWorking = true;
                new Thread(this::intakeUP).start();
                sleep(100);
                motorLiftL.setTargetPosition(IntakeConstants.LOW_JUNC);
                motorLiftR.setTargetPosition(IntakeConstants.LOW_JUNC);
                motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLiftL.setPower(1);
                motorLiftR.setPower(1);

            };
    Thread threadHighJ = new Thread(runHighJ),
            threadMidJ = new Thread(runMidJ),
            threadLowJ = new Thread(runLowJ), threadUp, threadDown;
    DistanceSensor distanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        motorLiftL = hardwareMap.get(DcMotorEx.class, "liftL");
        motorLiftR = hardwareMap.get(DcMotorEx.class,"liftR");
        servoLiftR = hardwareMap.get(Servo.class, "armR");
        servoLiftL = hardwareMap.get(Servo.class, "armL");
        claw = hardwareMap.get(Servo.class, "claw");
        servoKrutilka = hardwareMap.get(Servo.class, "servoKrutilka");
        servoLiftL.setDirection(Servo.Direction.REVERSE);
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        motorLiftR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLiftR.setTargetPositionTolerance(70);
        motorLiftL.setTargetPositionTolerance(70);
        motorLiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        waitForStart();
        if(opModeIsActive()){
            while(opModeIsActive()){
                boolean thrCheck = motorLiftR.getMode() != DcMotor.RunMode.RUN_TO_POSITION
                        && motorLiftL.getMode() != DcMotor.RunMode.RUN_TO_POSITION;
                if(motorLiftR.getCurrentPosition() < 2860 && motorLiftL.getCurrentPosition() < 2860) {
                    if (gamepad2.dpad_up) {
                        motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motorLiftL.setPower(1);
                        motorLiftR.setPower(1);
                    } else if (gamepad2.dpad_down) {
                        motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motorLiftL.setPower(-1);
                        motorLiftR.setPower(-1);
                    } else if (thrCheck && motorLiftL.getCurrentPosition() > 100 && motorLiftR.getCurrentPosition() > 100 && !thrWorking) {
                        motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        motorLiftL.setPower(0.003);
                        motorLiftR.setPower(0.003);
                    } else if(thrCheck){
                       telemetry.addLine("CHECK");
                        motorLiftL.setPower(0);
                        motorLiftR.setPower(0);
                    }
                } else if (gamepad2.dpad_down) {
                    motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorLiftL.setPower(-1);
                    motorLiftR.setPower(-1);
                } else {
                    motorLiftL.setPower(0);
                    motorLiftR.setPower(0);
                }
                if(gamepad2.right_stick_button){
                    if(threadUp == null || !threadUp.isAlive()){
                        threadUp = new Thread(runUP);
                        threadUp.start();
                    }
                } else if(gamepad2.b){
                    if(threadDown == null || !threadDown.isAlive()){
                        threadDown = new Thread(runDown);
                        threadDown.start();
                    }
                }
                if(gamepad2.y) {
                    if(threadHighJ == null || !threadHighJ.isAlive()){
                        threadHighJ = new Thread(runHighJ);
                        threadHighJ.start();
                    }
                } else if(gamepad2.x){
                    if(threadMidJ == null || !threadMidJ.isAlive()){
                        threadMidJ = new Thread(runMidJ);
                        threadMidJ.start();
                    }
                } else if(gamepad2.a){
                    if(threadLowJ == null || !threadLowJ.isAlive()){
                        threadLowJ = new Thread(runLowJ);
                        threadLowJ.start();
                    }
                }
                if(thrCheck && distanceSensor.getDistance(DistanceUnit.CM) < 5.5 && motorLiftL.getCurrentPosition() != 0 && motorLiftR.getCurrentPosition() != 0){
                    motorLiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorLiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                }
                telemetry.addLine("ServoL " + servoLiftL.getPosition());
                telemetry.addLine("ServoR " + servoLiftR.getPosition());
                telemetry.addLine("motorLift Left: " + motorLiftL.getCurrentPosition());
                telemetry.addLine("motorLift Right: " + motorLiftR.getCurrentPosition());
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
            claw.setPosition(0.68);
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

    /**
     * @param pos Position of the Servo
     */
    private void setServPosLift(double pos){
        //Right servo is not working
        servoLiftR.setPosition(pos);
        servoLiftL.setPosition(pos);
    }
}
