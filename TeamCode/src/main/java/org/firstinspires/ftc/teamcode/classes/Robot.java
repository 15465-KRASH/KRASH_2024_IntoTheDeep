package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public PinpointDrive drive;
    public Intake intake;
    public Lift lift;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Pose2d pose){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        drive = new PinpointDrive(hardwareMap, pose);
        intake = new Intake(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
    }

    public void deliverHigh(){
        intake.intakeOut();
        sleep(250);
        intake.setPackaged();
        sleep(500);
        intake.intakeOff();
        lift.elevatorPositionByConstant(Lift.LiftPositions.HIGH_BUCKET);
        sleep(2000);
        lift.setFlip();
        sleep(2000);
        lift.setFlipperReady();
        sleep(300);
        lift.elevatorPositionControl(0);
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
