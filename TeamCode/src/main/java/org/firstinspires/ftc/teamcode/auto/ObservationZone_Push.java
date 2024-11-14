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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@Autonomous(name = "Observation Zone Push", group = "Autonomous")
public class ObservationZone_Push extends LinearOpMode {


    public void runOpMode() throws InterruptedException{
        Pose2d initialPose = new Pose2d(14.5, -62.5, Math.toRadians(-90));
        Pose2d firstClipPose = new Pose2d(new Vector2d(8,-32.5), Math.toRadians(-90));
        Pose2d firstPushWaypoint = new Pose2d(new Vector2d(36, -36), Math.toRadians(90));
        Pose2d secondPushWaypoint = new Pose2d(new Vector2d(36, -24), Math.toRadians(90));
        Pose2d thirdPushWaypoint = new Pose2d(new Vector2d(40, -12), Math.toRadians(90));
        Pose2d fourthPushWaypoint = new Pose2d(new Vector2d(44, -24), Math.toRadians(90));
        Pose2d finalPushWaypoint = new Pose2d(new Vector2d(44, -50), Math.toRadians(90));

        Robot m_robot = new Robot(hardwareMap, telemetry, initialPose);

        m_robot.intake.setPackaged();

        TrajectoryActionBuilder placeFirst = m_robot.drive.actionBuilder(initialPose)
                .waitSeconds(1.5)
                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(firstClipPose.position, firstClipPose.heading)
                .waitSeconds(1);

        Action placeFirstAction = placeFirst.build();

        TrajectoryActionBuilder pushOne = placeFirst.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(firstPushWaypoint, Math.toRadians(90))
                .splineToLinearHeading(secondPushWaypoint, Math.toRadians(90))
                .splineToLinearHeading(thirdPushWaypoint, Math.toRadians(0))
                .splineToLinearHeading(fourthPushWaypoint, Math.toRadians(-90))
                .splineToLinearHeading(finalPushWaypoint, Math.toRadians(-90));

        Action pushOneAction = pushOne.build();


        telemetry.addLine("Starting Position");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        m_robot.lift.elevatorPositionByConstant(Lift.LiftPositions.HIGH_CLIP);

        Actions.runBlocking(placeFirstAction);

        m_robot.lift.autoClip(Lift.LiftPositions.HIGH_CLIP_RELEASE);
        m_robot.lift.elevatorPositionByConstant(Lift.LiftPositions.CLIP_PICKUP);

        Actions.runBlocking(pushOneAction);


        sleep(10);

    }
}