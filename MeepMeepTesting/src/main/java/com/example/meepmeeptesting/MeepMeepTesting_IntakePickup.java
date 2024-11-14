package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting_IntakePickup {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(18, 18)
                .build();

        Pose2d initialPose = new Pose2d(14.5, -62.5, Math.toRadians(-90));
        Pose2d firstClipPose = new Pose2d(new Vector2d(8,-32.5), Math.toRadians(-90));
        Pose2d grabOnePose = new Pose2d(new Vector2d(34, -38), Math.toRadians(40));
        Pose2d dropOnePose = new Pose2d(new Vector2d(40, -45), Math.toRadians(-40));
        Pose2d grabTwoPose = new Pose2d(new Vector2d(44, -38), Math.toRadians(40));


        TrajectoryActionBuilder placeFirst = myBot.getDrive().actionBuilder(initialPose)
                .waitSeconds(1.5)
                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(firstClipPose.position, firstClipPose.heading)
                .waitSeconds(1);

        TrajectoryActionBuilder grabOne = placeFirst.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineTo(grabOnePose.position, grabOnePose.heading)
                .lineToX(38);


        TrajectoryActionBuilder dropOne = grabOne.endTrajectory().fresh()
                .lineToX(40, new TranslationalVelConstraint(10))
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(-90))
//                .splineTo(dropOnePose.position, dropOnePose.heading)
                .splineToLinearHeading(dropOnePose, Math.toRadians(-90));

        TrajectoryActionBuilder grabTwo = dropOne.endTrajectory().fresh()
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(grabTwoPose, Math.toRadians(90))
                //Turn intake on here
//                .lineToX(grabTwoPose.position.x+2, new TranslationalVelConstraint(10))
                .waitSeconds(0.5);
//                .setTangent(-90)
//                .splineToLinearHeading(dropOnePose, Math.toRadians(-90));




//        TrajectoryActionBuilder finalPause = grabTwo.endTrajectory().fresh()
//                .waitSeconds(3)
//                .turn(Math.toRadians(1));





        myBot.runAction(
                new SequentialAction(
                        placeFirst.build(),
                        grabOne.build(),
                        dropOne.build(),
                        grabTwo.build()
//                        finalPause.build()

                ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}