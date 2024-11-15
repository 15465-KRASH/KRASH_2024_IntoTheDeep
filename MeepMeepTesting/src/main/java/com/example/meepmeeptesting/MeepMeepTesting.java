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

        Pose2d initialPose = new Pose2d(-14.5, -62.5, Math.toRadians(-90));
        Pose2d firstClipPose = new Pose2d(new Vector2d(-8,-32.5), Math.toRadians(-90));

        Pose2d parkWaypoint1 = new Pose2d(new Vector2d(-36, -30), Math.toRadians(-90));
        Pose2d parkWaypoint2 = new Pose2d(new Vector2d(-36, -20), Math.toRadians(-90));
        Pose2d parkWaypoint3 = new Pose2d(new Vector2d(-24, -12), Math.toRadians(-179.9));

//        Pose2d firstPickupWaypoint1 = new Pose2d(new Vector2d(-30, -33), Math.toRadians(155));
//
//        Pose2d dumpPose = new Pose2d(new Vector2d(-53,-54), Math.toRadians(45));
//
//        Pose2d secondPickupWaypoint1 = new Pose2d(new Vector2d(-42, -26), Math.toRadians(179.9));
//
//        Pose2d thirdPickupWaypoint1 = new Pose2d(new Vector2d(-50, -26), Math.toRadians(179.9));

//
//        Pose2d secondGrabWaypoint1 = new Pose2d(new Vector2d(35,-50), Math.toRadians(90));
//        Pose2d secondGrabWaypoint2 = new Pose2d(new Vector2d(35, -62.5), Math.toRadians(90));


        TrajectoryActionBuilder placeFirst = myBot.getDrive().actionBuilder(initialPose)
                .waitSeconds(0.75)
                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(firstClipPose.position, firstClipPose.heading)
                .waitSeconds(0.5);

        TrajectoryActionBuilder park = placeFirst.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                        .splineToLinearHeading(parkWaypoint1, Math.toRadians(90))
                .splineToLinearHeading(parkWaypoint3, Math.toRadians(0));



//        TrajectoryActionBuilder grabOne = placeFirst.endTrajectory().fresh()
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(firstPickupWaypoint1, firstPickupWaypoint1.heading)
//                       .lineToX(firstPickupWaypoint1.position.x-3);
//
//        TrajectoryActionBuilder dump1 = grabOne.endTrajectory().fresh()
//                .setTangent(-90)
//                .splineToLinearHeading(dumpPose, Math.toRadians(-135));
//
//        TrajectoryActionBuilder grabTwo = dump1.endTrajectory().fresh()
//                .setTangent(Math.toRadians(45))
//                .splineToLinearHeading(secondPickupWaypoint1, Math.toRadians(179.9));
//
//        TrajectoryActionBuilder dump2 = grabTwo.endTrajectory().fresh()
//                .setTangent(-90)
//                .splineToLinearHeading(dumpPose, Math.toRadians(-135));
//
//        TrajectoryActionBuilder grabThree = dump2.endTrajectory().fresh()
//                .setTangent(Math.toRadians(45))
//                .splineToLinearHeading(thirdPickupWaypoint1, Math.toRadians(179.9));
//




        myBot.runAction(
                new SequentialAction(
                        placeFirst.build(),
                        park.build()


                ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}