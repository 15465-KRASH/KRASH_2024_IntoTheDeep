package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(18, 18)
                .build();

        Pose2d initialPose = new Pose2d(14.5, -62.5, Math.toRadians(-90));
        Pose2d firstClipPose = new Pose2d(new Vector2d(8,-32.5), Math.toRadians(-90));
        Pose2d firstPushWaypoint = new Pose2d(new Vector2d(36, -36), Math.toRadians(90));
        Pose2d secondPushWaypoint = new Pose2d(new Vector2d(36, -24), Math.toRadians(90));
        Pose2d thirdPushWaypoint = new Pose2d(new Vector2d(40, -12), Math.toRadians(90));
        Pose2d fourthPushWaypoint = new Pose2d(new Vector2d(44, -24), Math.toRadians(90));
        Pose2d finalPushWaypoint = new Pose2d(new Vector2d(44, -50), Math.toRadians(90));




        TrajectoryActionBuilder placeFirst = myBot.getDrive().actionBuilder(initialPose)
                .waitSeconds(1.5)
                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(firstClipPose.position, firstClipPose.heading)
                .waitSeconds(1);

        TrajectoryActionBuilder pushOne = placeFirst.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(firstPushWaypoint, Math.toRadians(90))
                .splineToLinearHeading(secondPushWaypoint, Math.toRadians(90))
                .splineToLinearHeading(thirdPushWaypoint, Math.toRadians(0))
                .splineToLinearHeading(fourthPushWaypoint, Math.toRadians(-90))
                .splineToLinearHeading(finalPushWaypoint, Math.toRadians(-90));


//        TrajectoryActionBuilder dropOne = pushOne.endTrajectory().fresh()
//                .lineToX(40, new TranslationalVelConstraint(10))
//                .waitSeconds(0.5)
//                .setTangent(Math.toRadians(-90))
////                .splineTo(dropOnePose.position, dropOnePose.heading)
//                .splineToLinearHeading(dropOnePose, Math.toRadians(-90));






//        TrajectoryActionBuilder finalPause = grabTwo.endTrajectory().fresh()
//                .waitSeconds(3)
//                .turn(Math.toRadians(1));





        myBot.runAction(
                new SequentialAction(
                        placeFirst.build(),
                        pushOne.build()


                ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}