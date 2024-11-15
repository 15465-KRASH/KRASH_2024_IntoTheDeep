package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting_MainPush {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(18, 18)
                .build();

        Pose2d initialPose = new Pose2d(14.5, -62.5, Math.toRadians(-90));
        Pose2d firstClipPose = new Pose2d(new Vector2d(8,-32.5), Math.toRadians(-90));
        Pose2d firstPushWaypoint1 = new Pose2d(new Vector2d(36, -36), Math.toRadians(90));
        Pose2d firstPushWaypoint2 = new Pose2d(new Vector2d(36, -24), Math.toRadians(90));
        Pose2d firstPushWaypoint3 = new Pose2d(new Vector2d(42, -12), Math.toRadians(90));
        Pose2d firstPushWaypoint4 = new Pose2d(new Vector2d(50, -24), Math.toRadians(90));
        Pose2d firstPushWaypoint5 = new Pose2d(new Vector2d(50, -48), Math.toRadians(90));

        Pose2d secondPushWaypoint1 = new Pose2d(new Vector2d(42, -20), Math.toRadians(90));
        Pose2d secondPushWaypoint2 = new Pose2d(new Vector2d(52, -12), Math.toRadians(90));
        Pose2d secondPushWaypoint3 = new Pose2d(new Vector2d(58, -24), Math.toRadians(90));
        Pose2d secondPushWaypoint4 = new Pose2d(new Vector2d(58, -48), Math.toRadians(90));

        Pose2d firstGrabWaypoint1 = new Pose2d(new Vector2d(40, -40), Math.toRadians(90));
        Pose2d firstGrabWaypoint2 = new Pose2d(new Vector2d(35, -50), Math.toRadians(90));
        Pose2d firstGrabWaypoint3 = new Pose2d(new Vector2d(35, -62.5), Math.toRadians(90));

        Pose2d secondClipPose = new Pose2d(new Vector2d(5,-32.5), Math.toRadians(-89));

        Pose2d secondGrabWaypoint1 = new Pose2d(new Vector2d(35,-50), Math.toRadians(90));
        Pose2d secondGrabWaypoint2 = new Pose2d(new Vector2d(35, -62.5), Math.toRadians(90));


        TrajectoryActionBuilder placeFirst = myBot.getDrive().actionBuilder(initialPose)
                .waitSeconds(1.5)
                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(firstClipPose.position, firstClipPose.heading)
                .waitSeconds(1);

        TrajectoryActionBuilder pushOne = placeFirst.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(firstPushWaypoint1, Math.toRadians(90))
                .splineToLinearHeading(firstPushWaypoint2, Math.toRadians(90))
                .splineToLinearHeading(firstPushWaypoint3, Math.toRadians(0))
                .splineToLinearHeading(firstPushWaypoint4, Math.toRadians(-90))
                .splineToLinearHeading(firstPushWaypoint5, Math.toRadians(-90));

        TrajectoryActionBuilder pushTwo = pushOne.endTrajectory().fresh()
                .setTangent(90)
                .splineToLinearHeading(secondPushWaypoint1, Math.toRadians(90))
                .splineToLinearHeading(secondPushWaypoint2, Math.toRadians(0))
                .splineToLinearHeading(secondPushWaypoint3, Math.toRadians(-90))
                .splineToLinearHeading(secondPushWaypoint4, Math.toRadians(-90));

        TrajectoryActionBuilder pickupOne = pushTwo.endTrajectory().fresh()
                .setTangent(135)
                .splineToLinearHeading(firstGrabWaypoint1, Math.toRadians(-179.9))
                .splineToLinearHeading(firstGrabWaypoint2, Math.toRadians(-90))
                .splineToLinearHeading(firstGrabWaypoint3, Math.toRadians(-90));

        TrajectoryActionBuilder deliverOne = pickupOne.endTrajectory().fresh()
                .setTangent(90)
                .splineToLinearHeading(secondClipPose, Math.toRadians(90));

        TrajectoryActionBuilder pickupTwo = deliverOne.endTrajectory().fresh()
                .setTangent(-90)
                .splineToLinearHeading(secondGrabWaypoint1, Math.toRadians(-90))
                .splineToLinearHeading(secondGrabWaypoint2, Math.toRadians(-90));


//        TrajectoryActionBuilder finalPause = grabTwo.endTrajectory().fresh()
//                .waitSeconds(3)
//                .turn(Math.toRadians(1));





        myBot.runAction(
                new SequentialAction(
                        placeFirst.build(),
                        pushOne.build(),
                        pushTwo.build(),
                        pickupOne.build(),
                        deliverOne.build(),
                        pickupTwo.build()


                ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}