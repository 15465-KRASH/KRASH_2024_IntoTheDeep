package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "Observation Zone", group = "Autonomous")
public class ObservationZone extends LinearOpMode {


    public void runOpMode() throws InterruptedException{
        Pose2d initialPose = new Pose2d(14.5, -62.5, Math.toRadians(-90));
        Robot m_robot = new Robot(hardwareMap, telemetry, initialPose);

        m_robot.intake.setPackaged();

        TrajectoryActionBuilder placeFirst = m_robot.drive.actionBuilder(initialPose)
                .waitSeconds(1.5)
                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(6,-32), Math.toRadians(-90))
                .waitSeconds(1);

        Action placeFirstAction = placeFirst.build();

        TrajectoryActionBuilder grabOne = m_robot.drive.actionBuilder(new Pose2d(new Vector2d(6,-32), Math.toRadians(-90)))
                .setTangent(Math.toRadians(-90))
                .splineTo(new Vector2d(36, -36), Math.toRadians(45))
                .waitSeconds(1);

        Action grabOneAction = placeFirst.build();


        telemetry.addLine("Starting Position");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        m_robot.lift.elevatorPositionByConstant(Lift.LiftPositions.HIGH_CLIP);

        Actions.runBlocking(placeFirstAction);

        m_robot.lift.autoClip(Lift.LiftPositions.HIGH_CLIP_RELEASE);

        Actions.runBlocking(grabOneAction);

        m_robot.intake.intakeIn();
        m_robot.intake.setPickup();

        sleep(10);

    }
}