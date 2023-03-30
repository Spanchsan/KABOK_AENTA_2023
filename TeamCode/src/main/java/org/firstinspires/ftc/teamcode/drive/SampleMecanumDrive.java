package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.K;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Function;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.IntakeConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(5, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(2, 0, 0);

    public static double LATERAL_MULTIPLIER = 1.15;

    final double CLOSE_INTAKE = IntakeConstants.CLOSE_INTAKE,
            OPEN_INTAKE = IntakeConstants.OPEN_INTAKE,
            IDLE_INTAKE_POS = IntakeConstants.IDLE_INTAKE_POS;
    final double LOWEST_CONE_INTAKE = IntakeConstants.LOWEST_CONE_INTAKE;
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public DcMotorEx motorLift0, motorLift1;
    Servo sExtend1, sExtend2, sUpDownClaw1, sUpDownClaw2, sRotateClaw, sClaw;
    private List<DcMotorEx> motors;

    private IMU imu;
    private VoltageSensor batteryVoltageSensor;

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        // TODO: Adjust the orientations here to match your robot. See the FTC SDK documentation for
        // details
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        leftFront = hardwareMap.get(DcMotorEx.class, "motor1");
        leftRear = hardwareMap.get(DcMotorEx.class, "emotor2");
        rightRear = hardwareMap.get(DcMotorEx.class, "emotor3");
        rightFront = hardwareMap.get(DcMotorEx.class, "motor3");
        sExtend1 = hardwareMap.servo.get("serv0");
        sExtend2 = hardwareMap.servo.get("serv1");
        sUpDownClaw1 = hardwareMap.servo.get("serv2");
        sUpDownClaw2 = hardwareMap.servo.get("serv3");
        sRotateClaw = hardwareMap.servo.get("serv4");
        sClaw = hardwareMap.servo.get("serv5");
        motorLift0 = hardwareMap.get(DcMotorEx.class, "motor0");
        motorLift1 = hardwareMap.get(DcMotorEx.class, "emotor1");
        //motorLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // motorLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //motorLift1.setTargetPositionTolerance(15);
        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }


    /**
     * @param position position of the extension of intake
     *                 Position: 1, fully folded(to the robot)
     *                 Position: 0, fully extended
     */
    protected void setExtendIntake(double position){
        sExtend1.setPosition(position);
        sExtend2.setPosition(1 - position);
    }

    /**
     * @param position position of the extension of intake
     *                 Position: 0, fully upped(folded to the robot)
     *                 Position: 1, fully downed(extended)
     *                 position: 0.35 - IDLE
     *                 position: 0.73 - 5th highest cone(HIGH)
     *                 position: 0.75 - 4th highest cone
     *                 position: 0.79 - 3th highest cone
     *                 position: 0.85 - 2th highest cone
     *                 position: 0.92 - 1th highest cone(GROUND)
     */
    protected void setUpDownIntake(double position){
        sUpDownClaw1.setPosition(position);
        sUpDownClaw2.setPosition(position);
    }

    /**
     * @param position position of the rotation of the claw
     *                 Position: 0 - to grab the cone
     *                 position: 1 - to put on the basket
     */
    protected void setRotateClaw(double position){
        sRotateClaw.setPosition(position);
    }

    /**
     * @param position - position of the Claw
     *                 Position: 0 - hold the cone
     *                 Position: 0.7 - release the cone
     */
    protected void setClaw(double position){
        sClaw.setPosition(position);
    }

    /**
     * SETs Intake compartments to the IDLE position
     */
    public void IDLEIntakePosition(){
        setExtendIntake(1);
        setUpDownIntake(IDLE_INTAKE_POS);
        setRotateClaw(0);
        //sleep(300);
    }

    public void IDLEIntakePosition1(){
        setUpDownIntake(IDLE_INTAKE_POS);
        sleep(100);
        setRotateClaw(0);
    }

    /**
     * @param positionIntake - position of the Intake to be Set when at grabbing state
     * @param positionExtend - position of the Extend to be Set when at grabbing state
     */
    public void GRABIntakePosition(double positionIntake, double positionExtend){
        double overallTime = 750 * (1 - positionExtend);
        setClaw(OPEN_INTAKE);
        setExtendIntake(positionExtend);
        setUpDownIntake(0.5);
        sleep(100);
        setRotateClaw(0);
        sleep((long) (overallTime * 0.2));
        setUpDownIntake(positionIntake);
        sleep((long) (overallTime * 0.8));
        setClaw(CLOSE_INTAKE);
        sleep(450);
    }

    /**
     * Put cone to the Robot Basket
     * Standard position: UpDown Intake - 0.27
     *                    Extend Intake - 1
     */
    public void PUTCONEIntakePosition1(){
        double overallTime = Math.max(800, 1000 * (1 - sExtend1.getPosition()));
        setUpDownIntake(0.45);
        sleep(650);
        setExtendIntake(1);
        setRotateClaw(1);
        sleep((long) (overallTime * 0.6));
        setUpDownIntake(0.27);
        sleep((long) (overallTime * 0.4));
        setClaw(OPEN_INTAKE);
        sleep(200);
    }

    public void EXTENDIntakePosition(double positionIntake, double positionExtend){
        double overallTime = 750 * (1 - positionExtend);
        setClaw(OPEN_INTAKE);
        setExtendIntake(positionExtend);
        setUpDownIntake(0.5);
        sleep(100);
        setRotateClaw(0);
        sleep((long) (overallTime * 0.2));
        setUpDownIntake(positionIntake);
        sleep((long) (overallTime * 0.8));
    }

    public void EXTENDIntakePosition1(double positionIntake, double positionExtend){
        double overallTime = 750 * (1 - positionExtend);
        setClaw(OPEN_INTAKE);
        setUpDownIntake(0.7);
        sleep(400);
        setExtendIntake(positionExtend);
        sleep(100);
        setRotateClaw(0);
        sleep((long) (overallTime * 0.2));
        setUpDownIntake(positionIntake);
        sleep((long) (overallTime * 0.8));
    }

    public void GRAB_PUTCONEIntakePosition1(){
        double overallTime = Math.max(800, 940 * (1 - sExtend1.getPosition()));
        setClaw(CLOSE_INTAKE);
        sleep(450);
        setUpDownIntake(0.45);
        sleep(500);
        setExtendIntake(1);
        setRotateClaw(1);
        sleep((long) (overallTime * 0.6));
        setUpDownIntake(0.27);
        sleep((long) (overallTime * 0.4));
        setClaw(OPEN_INTAKE);
        sleep(200);
        IDLEIntakePosition1();
    }

    public void EXTEND_GRAB_PUTCONEIntakePosition1(double positionIntake, double positionExtend){
        EXTENDIntakePosition(positionIntake, positionExtend);
        GRAB_PUTCONEIntakePosition1();
    }

    public void EXTEND_GRAB_PUTCONEIntakePosition12(double positionIntake, double positionExtend){
        EXTENDIntakePosition1(positionIntake, positionExtend);
        GRAB_PUTCONEIntakePosition1();
    }

    public void sleep(long milliseconds){
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void throwCone(double power, int highestPos, int lowestPos, Telemetry telemetry, Function<Void, Boolean> func){
        motorLift1.setTargetPosition(highestPos);
        motorLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift1.setPower(power);
        motorLift0.setPower(power);
        while(motorLift1.isBusy()) {
            telemetry.addLine("" + motorLift1.getCurrentPosition());
            telemetry.addLine("Motor1: " + motorLift1.getPower() + " Motor0: " + motorLift0.getPower());
            telemetry.update();
        }
        //sleep(300);
        motorLift1.setTargetPosition(lowestPos);
        motorLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift1.setPower(1);
        motorLift0.setPower(-1);
        while(motorLift1.isBusy()){
            telemetry.addLine("" + motorLift1.getCurrentPosition());
            telemetry.addLine("Motor1: " + motorLift1.getPower() + " Motor0: " + motorLift0.getPower());
            telemetry.update();
        }
        motorLift1.setPower(0);
        motorLift0.setPower(0);
    }

    public void upCone(double power, int highPos, Telemetry telemetry, Function<Void, Boolean> func){
        motorLift1.setTargetPosition(highPos);
        motorLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift1.setPower(power);
        motorLift0.setPower(power);
        while(motorLift1.isBusy() && !func.apply(null)) {
            telemetry.addLine("" + motorLift1.getCurrentPosition());
            telemetry.addLine("Motor1: " + motorLift1.getPower() + " Motor0: " + motorLift0.getPower());
            telemetry.update();
        }
        motorLift1.setPower(0);
        motorLift0.setPower(0);
    }

    public void downCone(double power, double secs, Telemetry telemetry, Function<Void, Boolean> func){
        motorLift1.setPower(-power);
        motorLift0.setPower(-power);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while(runtime.seconds() <= secs && !func.apply(null)) {
            telemetry.addLine("" + motorLift1.getCurrentPosition());
            telemetry.addLine("Motor1: " + motorLift1.getPower() + " Motor0: " + motorLift0.getPower());
            telemetry.update();
        }
        motorLift1.setPower(0);
        motorLift0.setPower(0);
    }

    public void throwCone1(double power, double seconds, Telemetry telemetry){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        long idleT = 100;
        motorLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLift1.setPower(power);
        motorLift0.setPower(power);
        while(runtime.seconds() <= seconds) {
            telemetry.addLine("" + motorLift1.getCurrentPosition());
            telemetry.addLine("Motor1: " + motorLift1.getPower() + " Motor0: " + motorLift0.getPower());
            telemetry.update();
        }
        motorLift1.setPower(0);
        motorLift0.setPower(0);
        sleep(idleT);
        runtime.reset();
        motorLift1.setPower(-power);
        motorLift0.setPower(-power);
        while(runtime.seconds() <= seconds * 1.25) {
            telemetry.addLine("" + motorLift1.getCurrentPosition());
            telemetry.addLine("Motor1: " + motorLift1.getPower() + " Motor0: " + motorLift0.getPower());
            telemetry.update();
        }
        motorLift1.setPower(0);
        motorLift0.setPower(0);
    }

    public void throwCone2_1(double power, double seconds, Telemetry telemetry){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        motorLift1.setPower(power);
        motorLift0.setPower(power);
        while(runtime.seconds() <= seconds * 0.85) {
            telemetry.addLine("" + motorLift1.getCurrentPosition());
            telemetry.addLine("Motor1: " + motorLift1.getPower() + " Motor0: " + motorLift0.getPower());
            telemetry.update();
        }
//        motorLift1.setPower(0);
//        motorLift0.setPower(0);
    }

    public void throwCone2_2(double power, double seconds, Telemetry telemetry){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        motorLift1.setPower(power);
        motorLift0.setPower(power);
        while(runtime.seconds() <= seconds * 0.15) {
            telemetry.addLine("" + motorLift1.getCurrentPosition());
            telemetry.addLine("Motor1: " + motorLift1.getPower() + " Motor0: " + motorLift0.getPower());
            telemetry.update();
        }
        motorLift1.setPower(0);
        motorLift0.setPower(0);
        runtime.reset();
        motorLift1.setPower(-power);
        motorLift0.setPower(-power);
        while(runtime.seconds() <= seconds) {
            telemetry.addLine("" + motorLift1.getCurrentPosition());
            telemetry.addLine("Motor1: " + motorLift1.getPower() + " Motor0: " + motorLift0.getPower());
            telemetry.update();
        }
        motorLift1.setPower(0);
        motorLift0.setPower(0);
    }
}