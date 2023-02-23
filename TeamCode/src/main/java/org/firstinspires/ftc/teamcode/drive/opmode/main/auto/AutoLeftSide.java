package org.firstinspires.ftc.teamcode.drive.opmode.main.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.IntakeConstants;
import org.firstinspires.ftc.teamcode.util.TagDetector;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class AutoLeftSide extends LinearOpMode {

    private VoltageSensor batteryVoltageSensor;
    private TagDetector detector;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        detector = new TagDetector(hardwareMap, telemetry);
        TagDetector.Tag currTag = TagDetector.Tag.noTag;

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        drive.setPoseEstimate(new Pose2d(-37, -60, Math.toRadians(90)));
        Trajectory traj1= drive.trajectoryBuilder(new Pose2d(-37, -60, Math.toRadians(90)))
                .splineTo(new Vector2d(-37, -17), Math.toRadians(90))
                //.splineTo(new Vector2d(-37.2, -5.5), Math.toRadians(19))
                .build();
        drive.IDLEIntakePosition();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(-37.7, -3.9, Math.toRadians(17)))
                .build();
        //3.5
        while (!isStopRequested() && !isStarted()){
            TagDetector.Tag tag = detector.getTag();
            if(tag != TagDetector.Tag.noTag)
                currTag = tag;
            telemetry.addLine("Tag: " + tag.name());
            telemetry.update();
            sleep(40);
        }
        waitForStart();

        if (isStopRequested()) return;

        //drive.throwCone(1, 5, 0, telemetry, arg -> isStopRequested());
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.waitForIdle();
        //preloaded
        drive.throwCone1(0.9, 1.15 * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        //1st
        drive.GRABIntakePosition(0.73, 0.32);
        drive.PUTCONEIntakePosition1();
        drive.IDLEIntakePosition1();
        drive.throwCone1(0.9, 1.15 * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        //2nd
        drive.GRABIntakePosition(0.79, 0.32);
        drive.PUTCONEIntakePosition1();
        drive.IDLEIntakePosition1();
        drive.throwCone1(0.9, 1.15 * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        //3rd
        drive.GRABIntakePosition(0.875, 0.33);
        drive.PUTCONEIntakePosition1();
        drive.IDLEIntakePosition1();
        drive.throwCone1(0.9, 1.15 * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        //4th
        drive.GRABIntakePosition(0.89, 0.33);
        drive.PUTCONEIntakePosition1();
        drive.IDLEIntakePosition1();
        drive.throwCone1(0.9, 1.15 * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        //5th
        drive.GRABIntakePosition(0.92, 0.33);
        drive.PUTCONEIntakePosition1();
        drive.IDLEIntakePosition1();
        drive.throwCone1(0.9, 1.15 * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        if(currTag == TagDetector.Tag.left){
            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                    .splineTo(new Vector2d(-37, -14), Math.toRadians(0))
                    .splineTo(new Vector2d(-61, -14), Math.toRadians(0))
                    .build();
            drive.followTrajectory(traj3);
        } else if(currTag == TagDetector.Tag.mid){
            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                    .splineTo(new Vector2d(-37, -14), Math.toRadians(0))
                    .build();
            drive.followTrajectory(traj3);
        } else {
            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                    .splineTo(new Vector2d(-37, -14), Math.toRadians(0))
                    .splineTo(new Vector2d(-10, -14), Math.toRadians(0))
                    .build();
            drive.followTrajectory(traj3);
        }

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()){}
    }
}
