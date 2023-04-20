package org.firstinspires.ftc.teamcode.drive.opmode.main.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.IntakeConstants;
import org.firstinspires.ftc.teamcode.util.TagDetector;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class AutoLeftSideAcc extends LinearOpMode {

    private VoltageSensor batteryVoltageSensor;
    private TagDetector detector;
    private SampleMecanumDrive drive;
    int velToCone = 38;
    Vector2d poseForCone = new Vector2d(-29.5, -4.5);
    Vector2d poseForCone1 = new Vector2d(-29, -5.5);

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);
        detector = new TagDetector(hardwareMap, telemetry);
        TagDetector.Tag currTag = TagDetector.Tag.noTag;

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        drive.setPoseEstimate(new Pose2d(-36, -60, Math.toRadians(90)));
        //Первая траектория подьехать к junction-у в начале автономоки
        Trajectory traj1= drive.trajectoryBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
                .splineTo(new Vector2d(-37, -20), Math.toRadians(90))
                .splineTo(poseForCone1, Math.toRadians(48),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .addDisplacementMarker(1.5, () ->{
                    //new Thread(() -> raiseLift(telemetry)).start();
                    new Thread(() -> {
                        drive.intakeUP();
                    }).start();
                    new Thread(() -> {
                        drive.changePosLift(IntakeConstants.HIGH_JUNC + 65, 1, telemetry);
                    }).start();
                })
                .build();
        //Отьехать назад к пятиэтажным конусам
//        Trajectory trajToCon = drive.trajectoryBuilder(traj1.end(), true)
//                .splineTo(new Vector2d(37, -12), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(velToCone, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineTo(new Vector2d(60.5, -12), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(velToCone, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(1.5, () ->{
//                    new Thread(() -> {
//                        drive.intakeDOWN();
//                    }).start();
//                    new Thread(() -> drive.changePosLift(510, 1, telemetry)).start();
//                })
//                .build();
//        Trajectory trajToCon2 = drive.trajectoryBuilder(traj1.end(), true)
//                .splineTo(new Vector2d(37, -12), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(velToCone, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineTo(new Vector2d(60.5, -12), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(velToCone, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(1.5, () ->{
//                    new Thread(() -> {
//                        drive.intakeDOWN();
//                    }).start();
//                    new Thread(() -> drive.changePosLift(460, 1, telemetry)).start();
//                })
//                .build();
//        Trajectory trajToCon3 = drive.trajectoryBuilder(traj1.end(), true)
//                .splineTo(new Vector2d(37, -12), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(velToCone, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineTo(new Vector2d(59, -12), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(velToCone, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(1.5, () ->{
//                    new Thread(() -> {
//                        drive.intakeDOWN();
//                    }).start();
//                    new Thread(() -> drive.changePosLift(380, 1, telemetry)).start();
//                })
//                .build();
//        Trajectory trajToCon4 = drive.trajectoryBuilder(traj1.end(), true)
//                .splineTo(new Vector2d(37, -12), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(velToCone, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineTo(new Vector2d(59, -12), Math.toRadians(0),
//                        SampleMecanumDrive.getVelocityConstraint(velToCone, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(1.5, () ->{
//                    new Thread(() -> {
//                        drive.intakeDOWN();
//                    }).start();
//                    new Thread(() -> drive.changePosLift(300, 1, telemetry)).start();
//                })
//                .build();
////        //Вернуться к junction-у
//        Trajectory trajToJunc = drive.trajectoryBuilder(trajToCon.end())
//                .addDisplacementMarker(0.1, () ->{
//                    new Thread(() ->
//                            drive.changePosLift(IntakeConstants.HIGH_JUNC + 80, 1, telemetry)).start();
//                }).
//                addDisplacementMarker(1.5, () ->{
//                    new Thread(() -> drive.intakeUP()).start();
//                })
//                .splineTo(new Vector2d(39, -12), Math.toRadians(180-1e-6))
//                .splineTo(poseForCone1, Math.toRadians(132),
//                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(25))
//                .build();
        Trajectory trajToCon = getToConTraj(traj1.end(), 510);
        Trajectory trajToJunc = getToJuncTraj(trajToCon.end());
        Trajectory trajToCon2 = getToConTraj(trajToJunc.end(), 440);
        Trajectory trajToJunc2 = getToJuncTraj(trajToCon2.end());
        Trajectory trajToCon3 = getToConTraj(trajToJunc2.end(), 360);
        Trajectory trajToJunc3 = getToJuncTraj(trajToCon3.end());
        Trajectory trajToCon4 = getToConTraj(trajToJunc3.end(), 290);
        Trajectory trajToJunc4 = getToJuncTraj(trajToCon4.end());
        //3.5
        drive.claw.setPosition(IntakeConstants.CLOSE_INTAKE);
        sleep(1000);
        drive.setServPosLift(0.43);
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

        drive.followTrajectory(traj1);
        sleep(200);
        putCone();
        drive.followTrajectory(trajToCon);
        drive.claw.setPosition(IntakeConstants.CLOSE_INTAKE);
        sleep(300);
        drive.followTrajectory(trajToJunc);
        putCone();
        drive.followTrajectory(trajToCon2);
        drive.claw.setPosition(IntakeConstants.CLOSE_INTAKE);
        sleep(300);
        drive.followTrajectory(trajToJunc2);
        putCone();
        drive.followTrajectory(trajToCon3);
        drive.claw.setPosition(IntakeConstants.CLOSE_INTAKE);
        sleep(300);
        drive.followTrajectory(trajToJunc3);
        putCone();
//        drive.followTrajectory(trajToCon4);
//        drive.claw.setPosition(IntakeConstants.CLOSE_INTAKE);
//        sleep(300);
//        drive.followTrajectory(trajToJunc4);
//        putCone();
        new Thread(() ->
                drive.changePosLift(0, 1, telemetry)).start();
        new Thread(() -> drive.intakeDOWN()).start();
        if(currTag == TagDetector.Tag.left){
            Trajectory trajPark1 = drive.trajectoryBuilder(trajToJunc3.end(), true).
                    splineTo(new Vector2d(-29, -12), Math.toRadians(180-1e-6))
                    .build();
            Trajectory trajPark2 = drive.trajectoryBuilder(trajPark1.end())
                    .splineTo(new Vector2d(-10.5, -12), Math.toRadians(0))
                    .build();
            drive.followTrajectory(trajPark1);
            drive.followTrajectory(trajPark2);
        } else if(currTag == TagDetector.Tag.mid){
            Trajectory trajPark1 = drive.trajectoryBuilder(trajToJunc3.end(), true).
                    splineTo(new Vector2d(-32, -12), Math.toRadians(180-1e-6))
                    .build();
            drive.followTrajectory(trajPark1);
        } else {
            Trajectory trajPark1 = drive.trajectoryBuilder(trajToJunc3.end(), true).
                    splineTo(new Vector2d(-58, -12), Math.toRadians(180-1e-6))
                    .build();
            drive.followTrajectory(trajPark1);
        }
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()){}
    }

    private Trajectory getToConTraj(Pose2d pose, int lift){
        Trajectory trajToCon = drive.trajectoryBuilder(pose, true)
                .splineTo(new Vector2d(-37, -14), Math.toRadians(180-1e-6),
                        SampleMecanumDrive.getVelocityConstraint(velToCone, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(-61.5, -14), Math.toRadians(180-1e-6),
                        SampleMecanumDrive.getVelocityConstraint(velToCone, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(1.5, () ->{
                    new Thread(() -> {
                        drive.intakeDOWN();
                    }).start();
                    new Thread(() -> drive.changePosLift(lift, 1, telemetry)).start();
                })
                .build();
        return trajToCon;
    }

    private Trajectory getToJuncTraj(Pose2d pose){
        Trajectory trajToJunc = drive.trajectoryBuilder(pose)
                .addDisplacementMarker(0.1, () ->{
                    new Thread(() ->
                            drive.changePosLift(IntakeConstants.HIGH_JUNC + 20, 1, telemetry)).start();
                }).
                addDisplacementMarker(1.5, () ->{
                    new Thread(() -> drive.intakeUP()).start();
                })
                .splineTo(new Vector2d(-39, -12), Math.toRadians(0))
                .splineTo(poseForCone1, Math.toRadians(48),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .build();
        return trajToJunc;
    }


    private void putCone(){
        sleep(350);
        drive.setServPosLift(IntakeConstants.LIFT_PEREVOROT);
        sleep(120);
        drive.claw.setPosition(IntakeConstants.OPEN_INTAKE);
        sleep(200);
    }
}