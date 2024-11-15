package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.classes.Lift;
import org.firstinspires.ftc.teamcode.classes.Robot;

@Disabled
@Config
@Autonomous(name = "Observation Zone", group = "Autonomous")
public class ObservationZone extends LinearOpMode {


    public void runOpMode() throws InterruptedException{
        Pose2d initialPose = new Pose2d(14.5, -62.5, Math.toRadians(-90));
        Pose2d firstClipPose = new Pose2d(new Vector2d(8,-32.5), Math.toRadians(-90));
        Pose2d grabOnePose = new Pose2d(new Vector2d(34, -38), Math.toRadians(40));
        Pose2d dropOnePose = new Pose2d(new Vector2d(40, -45), Math.toRadians(-40));

        Robot m_robot = new Robot(hardwareMap, telemetry, initialPose);

        m_robot.intake.setPackaged();

        TrajectoryActionBuilder placeFirst = m_robot.drive.actionBuilder(initialPose)
                .waitSeconds(1.5)
                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(firstClipPose.position, firstClipPose.heading)
                .waitSeconds(1);

        Action placeFirstAction = placeFirst.build();

        TrajectoryActionBuilder grabOne = placeFirst.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineTo(grabOnePose.position, grabOnePose.heading)
                .lineToX(38);

        Action grabOneAction = new ParallelAction(
                grabOne.build(),
                m_robot.intake.delayedHeadDrop()
        );

        TrajectoryActionBuilder dropOne = grabOne.endTrajectory().fresh()
                .lineToX(40, new TranslationalVelConstraint(10))
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(dropOnePose, Math.toRadians(-90));

        Action dropOneAction = dropOne.build();


        telemetry.addLine("Starting Position");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        m_robot.lift.elevatorPositionByConstant(Lift.LiftPositions.HIGH_CLIP);

        Actions.runBlocking(placeFirstAction);

        m_robot.lift.autoClip(Lift.LiftPositions.HIGH_CLIP_RELEASE);
        m_robot.lift.elevatorPositionByConstant(Lift.LiftPositions.CLIP_PICKUP);

        Actions.runBlocking(grabOneAction);

        m_robot.intake.intakeIn();

        Actions.runBlocking(dropOneAction);
        m_robot.intake.intakeIn();
        m_robot.intake.setPackaged();




        sleep(10);

    }
}