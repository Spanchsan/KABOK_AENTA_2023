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
public class AutoTest extends LinearOpMode {

    private VoltageSensor batteryVoltageSensor;
    private TagDetector detector;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        detector = new TagDetector(hardwareMap, telemetry);
        TagDetector.Tag currTag = TagDetector.Tag.noTag;

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        drive.setPoseEstimate(new Pose2d(37, -60, Math.toRadians(90)));
        Trajectory traj1= drive.trajectoryBuilder(new Pose2d(37, -60, Math.toRadians(90)))
                .splineTo(new Vector2d(37, -17), Math.toRadians(90))
                //.splineTo(new Vector2d(-37.2, -5.5), Math.toRadians(19))
                .build();
        drive.IDLEIntakePosition();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(39.4, -2, Math.toRadians(163)))
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

        //drive.followTrajectory(traj1);
        //drive.followTrajectory(traj2);
        double powr = 1;
        double tim = 1.25;
        double extd = 0;
        drive.waitForIdle();
        drive.throwCone2_1(powr, tim * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        Thread threadTemp = new Thread(() -> drive.EXTEND_GRAB_PUTCONEIntakePosition12(0.91, extd+0.2));
        threadTemp.start();
        drive.throwCone2_2(powr, tim * 12.6 / batteryVoltageSensor.getVoltage(), telemetry);
        while(threadTemp.isAlive());
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()){}
    }
}
