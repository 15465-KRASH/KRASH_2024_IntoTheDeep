package org.firstinspires.ftc.teamcode.classes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Lift {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public DcMotorEx liftMotor;
    public Encoder liftEncoder;

    public DcMotorEx intakeMotor;

    public final double INTAKE_CURRENT_LIMIT = 2.0;

    public Servo flipper;
    public final double Flipper_Ready = 0.9;
    public final double Flipper_Deliver = 0.22;

    private int currentExt = 0;
    private int extStep = 5;
    private int minExt = 0;
    private int minSlowExt = 200;
    private int maxSlowExt = 3300;
    private int maxExt = 4900;

//    //Elevator PIDF Values
//    public static double KV = 0.5;
//    public static double KP = 15;
//    public static double KvP = 10;
//    public static double KvD = 0;
//    public static double KvI = 2.5;
//    public static double KF = 1.0;

    //Elevator PIDF Values worked for Bucket side
    public static double KV = 2.5;
    public static double KP = 25;
    public static double KvP = 12;
    public static double KvD = 0;
    public static double KvI = 2.5;
    public static double KF = 1.0;

    //Lift positions
    public static enum LiftPositions {
        HIGH_CLIP(2900),
        HIGH_CLIP_RELEASE(2150),
        LOW_CLIP(1000),
        LOW_CLIP_RELEASE(900),
        HIGH_BUCKET(4750),
        LOW_BUCKET(1875),
        CLIP_PICKUP(400),
        CLIP_PICKUP_DONE(475);

        public final int position;

        private LiftPositions(int position) {
            this.position = position;
        }
    }

    public Lift(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        flipper = hardwareMap.get(Servo.class, "flipper");

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setPIDFValues();

        liftEncoder = new OverflowEncoder(new RawEncoder(liftMotor));
        liftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor = hardwareMap.get(DcMotorEx.class,"backIntake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void changeLiftPos(double scale){
        currentExt = currentExt + (int)Math.round(extStep * scale);
        if (currentExt > maxExt) {
            currentExt = maxExt;
        } else if (currentExt < minExt){
            currentExt = minExt;
        }
    }

    public void setExtPosition(int target){
        liftMotor.setTargetPosition(target);
        liftMotor.setPower(1);
    }

    public int getCurrentExt(){
        return (int)liftEncoder.getPositionAndVelocity().position;
    }


    public void runLift(double power){
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double finalPower = limitPower(power, minExt, minSlowExt, maxExt, maxSlowExt, liftEncoder);
//        telemetry.addData("finalPower", finalPower);
        liftMotor.setPower(finalPower);
    }


    public double limitPower(double power, int min, int minSlow, int max, int maxSlow, Encoder encoder){
        double slow = 0.3;
        int pos = (int)encoder.getPositionAndVelocity().position;
//        if(isInRange(pos, minSlow, maxSlow)){
//            return power;
//        } else
//        telemetry.addData("pos", pos);
//        telemetry.addData("power", power);
        if ((power >= 0) && isInRange(pos, maxSlow, max)){
            return power*slow;
        } else if ((power >=0) && (pos >= max)){
            return 0;
        } else if ((power < 0) && isInRange(pos, min, minSlow)){
            return power*slow;
        } else if ((power < 0) && (pos <= min)){
            return 0;
        } else {
            return power;
        }
    }

    public boolean isInRange(int val, int min, int max){
        return (val >= min) && (val <= max);
    }


    public void elevatorPositionControl(int position) {
        liftMotor.setTargetPosition(position);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.9);
    }

    public void elevatorPositionByConstant(LiftPositions constant) {
        elevatorPositionControl(constant.position);
    }

    public void holdElevator() {
        int pos = liftMotor.getCurrentPosition();
        elevatorPositionControl(pos);
        if( pos < 0.5 * LiftPositions.CLIP_PICKUP.position){
            liftMotor.setPower(0);
        }
    }

    public void zeroLift() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getElevatorPosition() {
        return (int)liftEncoder.getPositionAndVelocity().position;
    }

    public boolean elevatorIsBusy() {
        return liftMotor.isBusy();
    }

    public double getElevatorPower() {
        return liftMotor.getPower();
    }

    public double getElevatorCurrent() {
        return liftMotor.getCurrent(CurrentUnit.AMPS);
    }

    public void setPIDFValues() {
        liftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(KP, 0, 0, 0));
        liftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(KvP, KvI, KvD, KV));
    }

    public void intakeIn(){
        intakeMotor.setPower(-1.0);
    }

    public void intakeOut(){
        intakeMotor.setPower(+0.7);
    }

    public void intakeOff(){
        intakeMotor.setPower(0);
    }

    public boolean intakeCurrentSpike(){
        double current = intakeMotor.getCurrent(CurrentUnit.AMPS);
        telemetry.addData("Back Intake Current", current);

        return current > INTAKE_CURRENT_LIMIT;
    }

    public void autoClip(LiftPositions position){
        elevatorPositionByConstant(position);
        sleep(800);
        intakeOut();
        sleep(200);
    }

    public void setFlipperReady(){
        flipper.setPosition(Flipper_Ready);
    }

    public void setFlip(){
        flipper.setPosition(Flipper_Deliver);
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public boolean safeToDump(){
        int pos = liftMotor.getCurrentPosition();
        return pos < 200;
    }

    public void timedFlip(){
        setFlip();
        sleep(1750);
        setFlipperReady();
        sleep(250);
        elevatorPositionControl(0);
    }

    //Actions
    public Action grabBlock() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeIn();
                    initialized = true;
                }

                if (intakeCurrentSpike()) {
                    elevatorPositionByConstant(LiftPositions.CLIP_PICKUP_DONE);
                    return false; //Return false to end
                } else {
                    return true;
                }
            }
        };
    }



}
