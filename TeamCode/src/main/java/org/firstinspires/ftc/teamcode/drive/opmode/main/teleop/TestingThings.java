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

    DcMotor motorFL, motorBL, motorFR, motorBR;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFL = hardwareMap.dcMotor.get("leftF");
        motorBL = hardwareMap.dcMotor.get("leftR");
        motorFR = hardwareMap.dcMotor.get("rightF");
        motorBR = hardwareMap.dcMotor.get("rightR");
        waitForStart();
        if(opModeIsActive()){
            while(opModeIsActive()){
                if(gamepad1.b){
                    motorFR.setPower(1);
                }else if(gamepad1.a){
                    motorBR.setPower(1);
                }else if(gamepad1.y){
                    motorFL.setPower(1);
                }else if(gamepad1.x){
                    motorBL.setPower(1);
                }
                telemetry.addLine("LEFT FRONT: " + motorFL.getCurrentPosition());
                telemetry.addLine("LEFT REAR: " + motorBL.getCurrentPosition());
                telemetry.addLine("RIGHT FRONT" + motorFR.getCurrentPosition());
                telemetry.addLine("RIGHT REAR: " + motorBR.getCurrentPosition());
//                telemetry.addLine("motorLift Left: " + motorLiftL.getCurrentPosition());
//                telemetry.addLine("motorLift Right: " + motorLiftR.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
