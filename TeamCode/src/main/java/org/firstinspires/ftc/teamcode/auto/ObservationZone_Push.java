package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
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

import org.firstinspires.ftc.teamcode.classes.HeadingStorage;
import org.firstinspires.ftc.teamcode.classes.Lift;
import org.firstinspires.ftc.teamcode.classes.Robot;

@Config
@Autonomous(name = "Observation Zone Push", group = "Autonomous", preselectTeleOp = "Drive")
public class ObservationZone_Push extends LinearOpMode {


    public void runOpMode() throws InterruptedException{
        Pose2d initialPose = new Pose2d(14.5, -62.5, Math.toRadians(-90));
        HeadingStorage.zeroOffset = initialPose.heading.log() - Math.toRadians(90);

        Pose2d firstClipPose = new Pose2d(new Vector2d(8,-32.5), Math.toRadians(-90));
        Pose2d firstPushWaypoint1 = new Pose2d(new Vector2d(36, -36), Math.toRadians(90));
        Pose2d firstPushWaypoint2 = new Pose2d(new Vector2d(36, -24), Math.toRadians(90));
        Pose2d firstPushWaypoint3 = new Pose2d(new Vector2d(42, -17), Math.toRadians(90));
        Pose2d firstPushWaypoint4 = new Pose2d(new Vector2d(50, -24), Math.toRadians(90));
        Pose2d firstPushWaypoint5 = new Pose2d(new Vector2d(50, -48), Math.toRadians(90));

        Pose2d secondPushWaypoint1 = new Pose2d(new Vector2d(42, -20), Math.toRadians(90));
        Pose2d secondPushWaypoint2 = new Pose2d(new Vector2d(52, -16), Math.toRadians(90));
        Pose2d secondPushWaypoint3 = new Pose2d(new Vector2d(58, -24), Math.toRadians(90));
        Pose2d secondPushWaypoint4 = new Pose2d(new Vector2d(58, -48), Math.toRadians(90));

        Pose2d firstGrabWaypoint1 = new Pose2d(new Vector2d(40, -40), Math.toRadians(90));
        Pose2d firstGrabWaypoint2 = new Pose2d(new Vector2d(35, -50), Math.toRadians(90));
        Pose2d firstGrabWaypoint3 = new Pose2d(new Vector2d(35, -62.5), Math.toRadians(90));

        Pose2d secondClipPose = new Pose2d(new Vector2d(5,-32.5), Math.toRadians(-89));

        Pose2d secondGrabWaypoint1 = new Pose2d(new Vector2d(35,-50), Math.toRadians(90));
        Pose2d secondGrabWaypoint2 = new Pose2d(new Vector2d(35, -62.5), Math.toRadians(90));

        Pose2d thirdClipPose = new Pose2d(new Vector2d(3,-32.5), Math.toRadians(-89));

        Pose2d parkPose = new Pose2d(new Vector2d(36,-60), Math.toRadians(0));

        Robot m_robot = new Robot(hardwareMap, telemetry, initialPose);

        m_robot.intake.setPackaged();

        TrajectoryActionBuilder placeFirst = m_robot.drive.actionBuilder(initialPose)
                .waitSeconds(0.7)
                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(firstClipPose.position, firstClipPose.heading)
                .waitSeconds(0.25);

        Action placeFirstAction = placeFirst.build();

        TrajectoryActionBuilder pushOne = placeFirst.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(firstPushWaypoint1, Math.toRadians(90))
                .splineToLinearHeading(firstPushWaypoint2, Math.toRadians(90))
                .splineToLinearHeading(firstPushWaypoint3, Math.toRadians(0))
                .splineToLinearHeading(firstPushWaypoint4, Math.toRadians(-90))
                .splineToLinearHeading(firstPushWaypoint5, Math.toRadians(-90));

        Action pushOneAction = pushOne.build();

        TrajectoryActionBuilder pushTwo = pushOne.endTrajectory().fresh()
                .setTangent(90)
                .splineToLinearHeading(secondPushWaypoint1, Math.toRadians(90))
                .splineToLinearHeading(secondPushWaypoint2, Math.toRadians(0))
                .splineToLinearHeading(secondPushWaypoint3, Math.toRadians(-90))
                .splineToLinearHeading(secondPushWaypoint4, Math.toRadians(-90));

        Action pushTwoAction = pushTwo.build();

        TrajectoryActionBuilder pickupOne = pushTwo.endTrajectory().fresh()
                .setTangent(135)
                .splineToLinearHeading(firstGrabWaypoint1, Math.toRadians(-179.9))
                .splineToLinearHeading(firstGrabWaypoint2, Math.toRadians(-90))
                .splineToLinearHeading(firstGrabWaypoint3, Math.toRadians(-90))
                .waitSeconds(0.5);

        Action pickupOneAction = pickupOne.build();

        TrajectoryActionBuilder secondClip = pickupOne.endTrajectory().fresh()
                .setTangent(90)
                .splineToLinearHeading(secondClipPose, Math.toRadians(90));

        Action secondClipAction = secondClip.build();

        TrajectoryActionBuilder pickupTwo = secondClip.endTrajectory().fresh()
                .setTangent(-90)
                .splineToLinearHeading(secondGrabWaypoint1, Math.toRadians(-90))
                .splineToLinearHeading(secondGrabWaypoint2, Math.toRadians(-90));

        Action pickupTwoAction = pickupTwo.build();

        TrajectoryActionBuilder thirdClip = pickupTwo.endTrajectory().fresh()
                .setTangent(90)
                .splineToLinearHeading(thirdClipPose, Math.toRadians(90));

        Action thirdClipAction = thirdClip.build();

        TrajectoryActionBuilder park = thirdClip.endTrajectory().fresh()
                .setTangent(-90)
                .splineToLinearHeading(parkPose, Math.toRadians(0), new TranslationalVelConstraint(60));

        Action parkAction = park.build();

        m_robot.lift.zeroLift();

        telemetry.addLine("Starting Position");
        telemetry.addData("Zero Offset", Math.toDegrees(HeadingStorage.zeroOffset));
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        m_robot.lift.elevatorPositionByConstant(Lift.LiftPositions.HIGH_CLIP);

        Actions.runBlocking(new ParallelAction(
                placeFirstAction,
                new SequentialAction(
                        new SleepAction(1),
                        new InstantAction(m_robot.intake::setHeadSafe)
                ))
        );

        m_robot.lift.autoClip(Lift.LiftPositions.HIGH_CLIP_RELEASE);
        m_robot.lift.elevatorPositionByConstant(Lift.LiftPositions.CLIP_PICKUP);
        m_robot.lift.intakeOff();

        Actions.runBlocking(pushOneAction);

        Actions.runBlocking(pushTwoAction);

        m_robot.lift.intakeIn();

        Actions.runBlocking(new ParallelAction(
                pickupOneAction,
                m_robot.lift.grabBlock()
        ));
//        sleep(500);

        m_robot.lift.intakeOff();
        m_robot.lift.elevatorPositionByConstant(Lift.LiftPositions.HIGH_CLIP);

        Actions.runBlocking(secondClipAction);

        m_robot.lift.autoClip(Lift.LiftPositions.HIGH_CLIP_RELEASE);
        m_robot.lift.elevatorPositionByConstant(Lift.LiftPositions.CLIP_PICKUP);
        m_robot.lift.intakeOff();


        m_robot.lift.intakeIn();
        Actions.runBlocking(new ParallelAction(
                pickupTwoAction,
                m_robot.lift.grabBlock()
        ));
//        sleep(500);
        m_robot.lift.intakeOff();
        m_robot.lift.elevatorPositionByConstant(Lift.LiftPositions.HIGH_CLIP);

        Actions.runBlocking(thirdClipAction);
        m_robot.lift.autoClip(Lift.LiftPositions.HIGH_CLIP_RELEASE);
        m_robot.lift.elevatorPositionByConstant(Lift.LiftPositions.CLIP_PICKUP);
        m_robot.lift.intakeOff();

        HeadingStorage.startingPose = parkPose;
        m_robot.drive.setCoast();
        Actions.runBlocking(parkAction);
        HeadingStorage.startingPose = m_robot.drive.pinpoint.getPositionRR();



    }
}