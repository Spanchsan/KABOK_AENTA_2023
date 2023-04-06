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
                .lineToLinearHeading(new Pose2d(-37.7, -3.9, Math.toRadians(14)))
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
        double powr = 1;
        double tim = 0.925;
        double extd = 0.285;
        drive.waitForIdle();
        // 1st
        drive.throwCone2_1(powr, tim * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        Thread threadTemp = new Thread(() -> drive.EXTEND_GRAB_PUTCONEIntakePosition1(0.75, extd));
        threadTemp.start();
        drive.throwCone2_2(powr, tim * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        while(threadTemp.isAlive());
        //2nd
        drive.throwCone2_1(powr, tim * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        threadTemp = new Thread(() -> drive.EXTEND_GRAB_PUTCONEIntakePosition1(0.79, extd));
        threadTemp.start();
        drive.throwCone2_2(powr, tim * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        while(threadTemp.isAlive());
        //3rd
        drive.throwCone2_1(powr, tim * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        threadTemp = new Thread(() -> drive.EXTEND_GRAB_PUTCONEIntakePosition1(0.86, extd));
        threadTemp.start();
        drive.throwCone2_2(powr, tim * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        while(threadTemp.isAlive());
        //4th
        drive.throwCone2_1(powr, tim * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        threadTemp = new Thread(() -> drive.EXTEND_GRAB_PUTCONEIntakePosition1(0.89, extd+0.1));
        threadTemp.start();
        drive.throwCone2_2(powr, tim * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        while(threadTemp.isAlive());
        //5th
        drive.throwCone2_1(powr, tim * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        threadTemp = new Thread(() -> drive.EXTEND_GRAB_PUTCONEIntakePosition1(0.92, extd+0.35));
        threadTemp.start();
        drive.throwCone2_2(powr, tim * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        while(threadTemp.isAlive());
        //6th
        drive.throwCone1(powr, tim * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        if(currTag == TagDetector.Tag.left){
            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                    .lineToLinearHeading(new Pose2d(-37, -13, Math.toRadians(180)))
                    .build();
            Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                    .splineTo(new Vector2d(-61, -13), Math.toRadians(180))
                    .build();
            drive.followTrajectory(traj3);
            drive.followTrajectory(traj4);
        } else if(currTag == TagDetector.Tag.mid){
            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                    .splineTo(new Vector2d(-37, -14), Math.toRadians(0))
                    .build();
            drive.followTrajectory(traj3);
        } else {
            Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                    .lineToLinearHeading(new Pose2d(-36, -14, Math.toRadians(0)))
                    .build();
            Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                    .splineTo(new Vector2d(-13.2, -14), Math.toRadians(0))
                    .build();
            drive.followTrajectory(traj3);
            drive.followTrajectory(traj4);
        }
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()){}
    }
}
