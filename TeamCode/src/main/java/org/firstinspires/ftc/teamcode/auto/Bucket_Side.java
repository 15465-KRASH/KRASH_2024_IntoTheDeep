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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.classes.HeadingStorage;
import org.firstinspires.ftc.teamcode.classes.Lift;
import org.firstinspires.ftc.teamcode.classes.Robot;

@Config
//@Disabled
@Autonomous(name = "Bucket Side", group = "Autonomous", preselectTeleOp = "Drive")
public class Bucket_Side extends LinearOpMode {


    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-33, -62.5, Math.toRadians(0));
        HeadingStorage.zeroOffset = initialPose.heading.log() - Math.toRadians(90);

        Pose2d veryfirstDumpPose = new Pose2d(new Vector2d(-52.1, -52.1), Math.toRadians(45));
        Pose2d firstDumpPose = new Pose2d(new Vector2d(-52.5, -52.5), Math.toRadians(50));

        Pose2d firstPickupWP1 = new Pose2d(new Vector2d(-48, -50), Math.toRadians(90));
        Pose2d firstPickupWP2 = new Pose2d(new Vector2d(-48, -40), Math.toRadians(90));

        Pose2d secondPickupWP1 = new Pose2d(new Vector2d(-52, -52), Math.toRadians(100));
        Pose2d secondPickupWP2 = new Pose2d(new Vector2d(-53, -45), Math.toRadians(100));

        Pose2d thirdPickupWP1 = new Pose2d(new Vector2d(-52, -52), Math.toRadians(120));
        Pose2d thirdPickupWP2 = new Pose2d(new Vector2d(-54, -43), Math.toRadians(120));


        Robot m_robot = new Robot(hardwareMap, telemetry, initialPose);

        m_robot.intake.setPackaged();

        TrajectoryActionBuilder dumpFirst = m_robot.drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(veryfirstDumpPose, Math.toRadians(-135));

        Action dumpFirstAction = dumpFirst.build();

        TrajectoryActionBuilder pickupFirst = dumpFirst.endTrajectory().fresh()
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(firstPickupWP1, Math.toRadians(90))
                .splineToLinearHeading(firstPickupWP2, Math.toRadians(90));

        Action pickupFirstAction = pickupFirst.build();

        TrajectoryActionBuilder dumpSecond = pickupFirst.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(firstDumpPose, Math.toRadians(-135));

        Action dumpSecondAction = dumpSecond.build();

        TrajectoryActionBuilder pickupSecond = dumpSecond.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .turnTo(secondPickupWP1.heading)
                .strafeTo(secondPickupWP2.position);

        Action pickupSecondAction = pickupSecond.build();

        TrajectoryActionBuilder dumpThird = pickupSecond.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(firstDumpPose, Math.toRadians(-135));

        Action dumpThirdAction = dumpThird.build();

        TrajectoryActionBuilder pickupThird = dumpThird.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .turnTo(thirdPickupWP1.heading)
                .strafeTo(thirdPickupWP2.position);

        Action pickupThirdAction = pickupThird.build();

        TrajectoryActionBuilder dumpFourth = pickupThird.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(firstDumpPose, Math.toRadians(-135))
                .turnTo(firstDumpPose.heading);

        Action dumpFourthAction = dumpThird.build();


        m_robot.lift.zeroLift();

        telemetry.addLine("Starting Position");
        telemetry.addData("Zero Offset", Math.toDegrees(HeadingStorage.zeroOffset));
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        //Score Preload
        m_robot.intake.setExtPosition(m_robot.intake.safeExt);
        sleep(0);

        m_robot.lift.elevatorPositionByConstant(Lift.LiftPositions.HIGH_BUCKET);

        Actions.runBlocking(dumpFirstAction);
        m_robot.intake.setPickup();

        sleep(900);

        m_robot.lift.timedFlip();

        //Pickup First
        m_robot.intake.intakeIn();

        Actions.runBlocking(pickupFirstAction);
        m_robot.intake.setPackaged();
        m_robot.intake.intakeOff();


        //Score second
        Actions.runBlocking(new ParallelAction(
                dumpSecondAction,
                new SequentialAction(
                        m_robot.intake.deliverToDump(),
                        new InstantAction(() -> m_robot.lift.elevatorPositionByConstant(Lift.LiftPositions.HIGH_BUCKET))
                )
        ));

        m_robot.intake.intakeOff();
        sleep(2000);
        m_robot.intake.setPickup();
        m_robot.intake.setExtPosition(m_robot.intake.safeExt-50);
        m_robot.lift.timedFlip();

        //Pickup second
        m_robot.intake.intakeIn();
        Actions.runBlocking(new ParallelAction(
                pickupSecondAction,
                new InstantAction(() -> m_robot.intake.setExtPosition(m_robot.intake.safeExt+100))
        ));
        m_robot.intake.setPackaged();
        m_robot.intake.intakeOff();

        //Score third
        Actions.runBlocking(new ParallelAction(
                dumpThirdAction,
                new SequentialAction(
                        m_robot.intake.deliverToDump(),
                        new InstantAction(() -> m_robot.lift.elevatorPositionByConstant(Lift.LiftPositions.HIGH_BUCKET))
                )
        ));

        m_robot.intake.intakeOff();
        sleep(2000);
        m_robot.intake.setPickup();
        m_robot.intake.setExtPosition(m_robot.intake.safeExt-50);
        m_robot.lift.timedFlip();

        //Pickup third
        m_robot.intake.intakeIn();
        Actions.runBlocking(new ParallelAction(
                pickupThirdAction,
                new InstantAction(() -> m_robot.intake.setExtPosition(m_robot.intake.safeExt+100))
        ));
        m_robot.intake.setPackaged();
        m_robot.intake.intakeOff();

        Actions.runBlocking(new ParallelAction(
                dumpFourthAction,
                new SequentialAction(
                        m_robot.intake.deliverToDump(),
                        new InstantAction(() -> m_robot.lift.elevatorPositionByConstant(Lift.LiftPositions.HIGH_BUCKET))
                )
        ));

        HeadingStorage.startingPose = m_robot.drive.pinpoint.getPositionRR();
        m_robot.intake.intakeOff();
        sleep(2000);
        m_robot.intake.setExtPosition(m_robot.intake.safeExt-50);
        m_robot.lift.timedFlip();

        sleep(2);



    }


}