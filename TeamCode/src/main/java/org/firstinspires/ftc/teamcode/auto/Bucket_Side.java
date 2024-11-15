package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.classes.Lift;
import org.firstinspires.ftc.teamcode.classes.Robot;

@Config
@Autonomous(name = "Bucket Side", group = "Autonomous")
public class Bucket_Side extends LinearOpMode {


    public void runOpMode() throws InterruptedException{
        Pose2d initialPose = new Pose2d(-14.5, -62.5, Math.toRadians(-90));
        Pose2d firstClipPose = new Pose2d(new Vector2d(-8,-32.5), Math.toRadians(-90));

        Pose2d parkWaypoint1 = new Pose2d(new Vector2d(-36, -30), Math.toRadians(-90));
        Pose2d parkWaypoint2 = new Pose2d(new Vector2d(-36, -20), Math.toRadians(-90));
        Pose2d parkWaypoint3 = new Pose2d(new Vector2d(-24, -12), Math.toRadians(-179.9));

//        Pose2d firstPickupWaypoint1 = new Pose2d(new Vector2d(-31, -34.5), Math.toRadians(155));
//
//        Pose2d dumpPose = new Pose2d(new Vector2d(-53,-53), Math.toRadians(50));
//
//        Pose2d secondPickupWaypoint1 = new Pose2d(new Vector2d(-42, -26), Math.toRadians(-179.9));
//        Pose2d secondPickupWaypoint2 = new Pose2d(new Vector2d(-42, -26), Math.toRadians(-179.9));
//
//        Pose2d thirdPickupWaypoint1 = new Pose2d(new Vector2d(-50, -26), Math.toRadians(-179.9));


        Robot m_robot = new Robot(hardwareMap, telemetry, initialPose);

        m_robot.intake.setPackaged();

        TrajectoryActionBuilder placeFirst = m_robot.drive.actionBuilder(initialPose)
                .waitSeconds(0.7)
                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(firstClipPose.position, firstClipPose.heading)
                .waitSeconds(0.25);

        Action placeFirstAction = placeFirst.build();

        TrajectoryActionBuilder park = placeFirst.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(parkWaypoint1.position, Math.toRadians(90))
                .splineToLinearHeading(parkWaypoint3, Math.toRadians(0));


        Action parkAction = placeFirst.build();

//        TrajectoryActionBuilder grabOne = placeFirst.endTrajectory().fresh()
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(firstPickupWaypoint1, firstPickupWaypoint1.heading)
//                .lineToX(firstPickupWaypoint1.position.x-3);
//
//        Action grabOneAction = new ParallelAction(
//                grabOne.build(),
//                m_robot.intake.delayedHeadDrop()
//        );
//
//        TrajectoryActionBuilder dump1 = grabOne.endTrajectory().fresh()
//                .setTangent(-90)
//                .splineToLinearHeading(dumpPose, Math.toRadians(-125));
//
//        Action dump1Action = new ParallelAction(
//                new SequentialAction(
//                        new SleepAction(0.5),
//                        dump1.build()
//                ),
//                m_robot.intake.deliverToDump()
//        );
//
//        TrajectoryActionBuilder grabTwo = dump1.endTrajectory().fresh()
//                .setTangent(Math.toRadians(45))
//                .splineToLinearHeading(secondPickupWaypoint1, Math.toRadians(-179.9));
//
//        Action grabTwoAction = new ParallelAction(
//                grabOne.build(),
//                m_robot.intake.delayedHeadDrop()
//        );
//
//        TrajectoryActionBuilder dump2 = grabTwo.endTrajectory().fresh()
//                .setTangent(-90)
//                .splineToLinearHeading(dumpPose, Math.toRadians(-125));
//
//        Action dump2Action = new ParallelAction(
//                new SequentialAction(
//                        new SleepAction(0.5),
//                        dump2.build()
//                ),
//                m_robot.intake.deliverToDump()
//        );
//
//        TrajectoryActionBuilder grabThree = dump2.endTrajectory().fresh()
//                .setTangent(Math.toRadians(45))
//                .splineToLinearHeading(thirdPickupWaypoint1, Math.toRadians(-179.9));
//
//        Action grabThreeAction = new ParallelAction(
//                grabOne.build(),
//                m_robot.intake.delayedHeadDrop()
//        );
//
//        TrajectoryActionBuilder dump3 = grabThree.endTrajectory().fresh()
//                .setTangent(-90)
//                .splineToLinearHeading(dumpPose, Math.toRadians(-125));
//
//        Action dump3Action = new ParallelAction(
//                new SequentialAction(
//                        new SleepAction(0.5),
//                        dump3.build()
//                ),
//                m_robot.intake.deliverToDump()
//        );



        telemetry.addLine("Starting Position");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        m_robot.lift.elevatorPositionByConstant(Lift.LiftPositions.HIGH_CLIP);

        Actions.runBlocking(placeFirstAction);

        m_robot.lift.autoClip(Lift.LiftPositions.HIGH_CLIP_RELEASE);
        m_robot.lift.elevatorPositionControl(0);
        m_robot.lift.intakeOff();

        Actions.runBlocking(parkAction);
        m_robot.lift.setFlip();
        sleep(3000);

//        //First pickup and deliver
//        m_robot.intake.intakeIn();
//        Actions.runBlocking(grabOneAction);
//
//        m_robot.intake.intakeIn();
//
//        Actions.runBlocking(dump1Action);
//        m_robot.deliverHigh();
//
//        //Second pickup and deliver
//        Actions.runBlocking(grabTwoAction);
//
//        m_robot.intake.intakeIn();
//
//        Actions.runBlocking(dump2Action);
//        m_robot.deliverHigh();

//        //Third pickup and deliver
//        Actions.runBlocking(grabThreeAction);
//
//        m_robot.intake.intakeIn();
//
//        Actions.runBlocking(dump3Action);
//        m_robot.deliverHigh();









    }
}