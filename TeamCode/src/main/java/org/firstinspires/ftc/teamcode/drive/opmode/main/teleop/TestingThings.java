package org.firstinspires.ftc.teamcode.drive.opmode.main.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "PIS'KA")
public class TestingThings extends LinearOpMode {

    DistanceSensor distanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        waitForStart();
        if(opModeIsActive()){
            while(opModeIsActive()){
                telemetry.addLine("Distance: " + distanceSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
        }
    }
}
