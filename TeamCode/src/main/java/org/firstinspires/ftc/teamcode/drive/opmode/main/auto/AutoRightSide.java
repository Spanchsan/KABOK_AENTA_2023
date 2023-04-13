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
public class AutoRightSide extends LinearOpMode {

    private VoltageSensor batteryVoltageSensor;
    private TagDetector detector;
    private SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

            drive = new SampleMecanumDrive(hardwareMap);
//        detector = new TagDetector(hardwareMap, telemetry);
//        TagDetector.Tag currTag = TagDetector.Tag.noTag;

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        drive.setPoseEstimate(new Pose2d(36, -60, Math.toRadians(90)));
        //Первая траектория подьехать к junction-у в начале автономоки
        Trajectory traj1= drive.trajectoryBuilder(new Pose2d(36, -60, Math.toRadians(90)))
                .splineTo(new Vector2d(37, -20), Math.toRadians(90))
                .splineTo(new Vector2d(28, -4), Math.toRadians(130))
                .build();
        //Отьехать назад к пятиэтажным конусам
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), true)
                .splineTo(new Vector2d(37, -12), Math.toRadians(0))
                .build();
        Trajectory traj21 = drive.trajectoryBuilder(traj2.end(), true)
                .splineTo(new Vector2d(60, -12), Math.toRadians(0))
                .build();
        //Вернуться к junction-у
        Trajectory traj3 = drive.trajectoryBuilder(traj21.end())
                .splineTo(new Vector2d(37, -12), Math.toRadians(180-1e-6))
                .build();
        Trajectory traj31 = drive.trajectoryBuilder(traj3.end())
        .splineTo(new Vector2d(28, -4), Math.toRadians(130))
                .build();
        //3.5

//        while (!isStopRequested() && !isStarted()){
//            TagDetector.Tag tag = detector.getTag();
//            if(tag != TagDetector.Tag.noTag)
//                currTag = tag;
//            telemetry.addLine("Tag: " + tag.name());
//            telemetry.update();
//            sleep(40);
//        }
        drive.claw.setPosition(IntakeConstants.CLOSE_INTAKE);
        sleep(1000);
        drive.setServPosLift(0.42);
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traj1);
        putCone(telemetry);
        drive.followTrajectory(traj2);
        new Thread(() ->{
            drive.runDown.run();
            drive.changePosLift(650, 1, telemetry);
        }).start();
        drive.followTrajectory(traj21);
        drive.claw.setPosition(IntakeConstants.CLOSE_INTAKE);
        sleep(400);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj31);
        putCone(telemetry);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()){}
    }

    private void putCone(Telemetry telemetry){
        drive.runUP.run();
        drive.changePosLift(IntakeConstants.HIGH_JUNC, 1, telemetry);
        sleep(500);
        drive.setServPosLift(IntakeConstants.LIFT_PEREVOROT);
        sleep(300);
        drive.claw.setPosition(IntakeConstants.OPEN_INTAKE);
        sleep(500);
    }
}