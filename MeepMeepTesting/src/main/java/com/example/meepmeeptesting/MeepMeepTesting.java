package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting{
    static MeepMeep meepMeep = new MeepMeep(800);
    public static void main(String[] args) {

        RoadRunnerBotEntity myBot1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.77).build();
        myBot1.getDrive().setPoseEstimate(new Pose2d(36, -60, Math.toRadians(90)));
        TrajectorySequence traj1 = myBot1.getDrive().
                trajectorySequenceBuilder(new Pose2d(36, -60, Math.toRadians(90)))
                .splineTo(new Vector2d(36, -20), Math.toRadians(90))
                .splineTo(new Vector2d(32, -10), Math.toRadians(130))
                //to Cone
                .setReversed(true)
                .splineTo(new Vector2d(37, -12), Math.toRadians(0))
                .splineTo(new Vector2d(57, -12), Math.toRadians(0))
                //to junction
                .setReversed(false)
                .splineTo(new Vector2d(37, -12), Math.toRadians(180-1e-6))
                .splineTo(new Vector2d(32, -10), Math.toRadians(130))
                //to Cone
                .setReversed(true)
                .splineTo(new Vector2d(37, -12), Math.toRadians(0))
                .splineTo(new Vector2d(57, -12), Math.toRadians(0))
                //to junction
                .setReversed(false)
                .splineTo(new Vector2d(37, -12), Math.toRadians(180-1e-6))
                .splineTo(new Vector2d(32, -10), Math.toRadians(130))
                //to Cone
                .setReversed(true)
                .splineTo(new Vector2d(37, -12), Math.toRadians(0))
                .splineTo(new Vector2d(57, -12), Math.toRadians(0))
                //to junction
                .setReversed(false)
                .splineTo(new Vector2d(37, -12), Math.toRadians(180-1e-6))
                .splineTo(new Vector2d(32, -10), Math.toRadians(130))
                //to Cone
                .setReversed(true)
                .splineTo(new Vector2d(37, -12), Math.toRadians(0))
                .splineTo(new Vector2d(57, -12), Math.toRadians(0))
                //to junction
                .setReversed(false)
                .splineTo(new Vector2d(37, -12), Math.toRadians(180-1e-6))
                .splineTo(new Vector2d(32, -10), Math.toRadians(130))
                //to Cone
                .setReversed(true)
                .splineTo(new Vector2d(37, -12), Math.toRadians(0))
                .splineTo(new Vector2d(57, -12), Math.toRadians(0))
                //to junction
                .setReversed(false)
                .splineTo(new Vector2d(37, -12), Math.toRadians(180-1e-6))
                .splineTo(new Vector2d(32, -10), Math.toRadians(130))
                //leftx
                .setReversed(true)
                .splineTo(new Vector2d(37, -12), Math.toRadians(0))
                .setReversed(false)
                .splineTo(new Vector2d(12, -13), Math.toRadians(180-1e-6))
                .build();
        myBot1.followTrajectorySequence(traj1);

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.77).build();
        myBot2.getDrive().setPoseEstimate(new Pose2d(-36, -60, Math.toRadians(90)));
        TrajectorySequence traj2 = myBot2.getDrive().
                trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
                .splineTo(new Vector2d(-36, -20), Math.toRadians(90))
                .splineTo(new Vector2d(-32, -10), Math.toRadians(50))
                //to Cone
                .setReversed(true)
                .splineTo(new Vector2d(-37, -12), Math.toRadians(180-1e-6))
                .splineTo(new Vector2d(-57, -12), Math.toRadians(180-1e-6))
                //to junction
                .setReversed(false)
                .splineTo(new Vector2d(-37, -12), Math.toRadians(0))
                .splineTo(new Vector2d(-32, -10), Math.toRadians(50))
                //to Cone
                .setReversed(true)
                .splineTo(new Vector2d(-37, -12), Math.toRadians(180-1e-6))
                .splineTo(new Vector2d(-57, -12), Math.toRadians(180-1e-6))
                //to junction
                .setReversed(false)
                .splineTo(new Vector2d(-37, -12), Math.toRadians(0))
                .splineTo(new Vector2d(-32, -10), Math.toRadians(50))
                //to Cone
                .setReversed(true)
                .splineTo(new Vector2d(-37, -12), Math.toRadians(180-1e-6))
                .splineTo(new Vector2d(-57, -12), Math.toRadians(180-1e-6))
                //to junction
                .setReversed(false)
                .splineTo(new Vector2d(-37, -12), Math.toRadians(0))
                .splineTo(new Vector2d(-32, -10), Math.toRadians(50))
                //to Cone
                .setReversed(true)
                .splineTo(new Vector2d(-37, -12), Math.toRadians(180-1e-6))
                .splineTo(new Vector2d(-57, -12), Math.toRadians(180-1e-6))
                //to junction
                .setReversed(false)
                .splineTo(new Vector2d(-37, -12), Math.toRadians(0))
                .splineTo(new Vector2d(-32, -10), Math.toRadians(50))
                //to Cone
                .setReversed(true)
                .splineTo(new Vector2d(-37, -12), Math.toRadians(180-1e-6))
                .splineTo(new Vector2d(-57, -12), Math.toRadians(180-1e-6))
                //to junction
                .setReversed(false)
                .splineTo(new Vector2d(-37, -12), Math.toRadians(0))
                .splineTo(new Vector2d(-32, -10), Math.toRadians(50))
                //leftx
                .setReversed(true)
                .splineTo(new Vector2d(-37, -12), Math.toRadians(180-1e-6))
                .setReversed(false)
                .splineTo(new Vector2d(-12, -13), Math.toRadians(0))
                .build();
        myBot2.followTrajectorySequence(traj2);



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot1)
                .addEntity(myBot2)
                .start();
    }
}
