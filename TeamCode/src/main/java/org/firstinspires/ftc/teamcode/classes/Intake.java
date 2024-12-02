package org.firstinspires.ftc.teamcode.classes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    LED topLED_red;
    LED topLED_green;
    public Action currentLEDAction;

    public enum LEDMode {
        SOLID,
        OFF,
        SLOW_BLINK,
        FAST_BLINK,
        GREEN,
        AMBER;
    }

    public LEDMode ledMode = LEDMode.OFF;

    ElapsedTime blinkTimer;

    boolean ledStop = true;

    public CRServo headMotor;
    public Servo headRotLeft, headRotRight;
    public Servo fourBarLeft, fourBarRight;
    public DcMotorEx extensionMotor;
    public Encoder extensionEncoder;

    public RevColor revColor;

    public int currentExt = 0;
    public int extStep = 5;
    public int minExt = 500;
    public int minSlowExt = 405;
    public int maxSlowExt = 1000;
    public int maxExt = 1140;
    public int safeExt = 700;

    private int pickupExtension = 1000;
    private int dumpExtension = 500;

    public class HeadPos {
        double headRotLeftPos;
        double headRotRightPos;
        double fourBarLeftPos;
        double fourBarRightPos;

        public HeadPos(double headRotLeftPos, double headRotRightPos, double fourBarLeftPos, double fourBarRightPos){
            this.headRotLeftPos = headRotLeftPos;
            this.headRotRightPos = headRotRightPos;
            this.fourBarLeftPos = fourBarLeftPos;
            this.fourBarRightPos = fourBarRightPos;
        }
    }

    public enum HeadPosition {
        PICKUP,
        DUMP,
        COBRA,
        HOLD,
        LOWPRO;
    }

    public HeadPosition headPosition = HeadPosition.COBRA;

    private HeadPos pickupPos = new HeadPos(0.70, 0.70, 0.65, 0.65);
    private HeadPos dumpPos = new HeadPos(0.1, 0.1, 0.05, 0.05);
    private HeadPos packagePos = new HeadPos(0.55, 0.55, 0.05, 0.05);
    private HeadPos holdPos = new HeadPos(0.65, 0.65, 0.05, 0.05);
    private HeadPos lowProPos = new HeadPos(0.5, 0.5, 0.4, 0.4);




    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        revColor = new RevColor(hardwareMap, telemetry);

        topLED_green = hardwareMap.get(LED.class, "top_led_green");
        topLED_red = hardwareMap.get(LED.class, "top_led_red");

        topLED_green.off();
        topLED_red.off();

        headMotor = hardwareMap.get(CRServo.class, "headMotor");
        headRotLeft = hardwareMap.get(Servo.class, "headRotLeft");
        headRotLeft.setDirection(Servo.Direction.REVERSE);
        headRotRight = hardwareMap.get(Servo.class, "headRotRight");
        fourBarLeft = hardwareMap.get(Servo.class, "fourBarLeft");
        fourBarLeft.setDirection(Servo.Direction.REVERSE);
        fourBarRight = hardwareMap.get(Servo.class, "fourBarRight");
        extensionMotor = hardwareMap.get(DcMotorEx.class, "extensionMotor");

        extensionMotor.setTargetPosition(0);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        extensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        extensionEncoder = new OverflowEncoder(new RawEncoder(extensionMotor));
        extensionEncoder.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void changeExtension(double scale){
        currentExt = currentExt + (int)Math.round(extStep * scale);
        if (currentExt > maxExt) {
            currentExt = maxExt;
        } else if (currentExt < minExt){
            currentExt = minExt;
        }
    }

    public void setExtPosition(int target){
        extensionMotor.setTargetPosition(target);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setPower(1);
    }

    public void setDumpExt(){
        setExtPosition(minExt);
    }

    public double getCurrentExt(){
        return extensionEncoder.getPositionAndVelocity().position;
    }

    public void setHeadPos(HeadPos headPos){
        headRotRight.setPosition(headPos.headRotRightPos);
        headRotLeft.setPosition(headPos.headRotLeftPos);
        fourBarRight.setPosition(headPos.fourBarRightPos);
        fourBarLeft.setPosition(headPos.fourBarLeftPos);
    }

    public void setDump(){
        setHeadPos(dumpPos);
        headPosition = HeadPosition.DUMP;
    }

    public void setPickup(){
        setHeadPos(pickupPos);
        headPosition = HeadPosition.PICKUP;
    }

    public void setPackaged(){
        setHeadPos(packagePos);
        headPosition = HeadPosition.COBRA;
    }

    public HeadPosition getHeadPosition(){
        return headPosition;
    }

    public boolean liftSafe(){
        return headPosition != HeadPosition.DUMP;
    }

    public void intakeIn(){
        headMotor.setPower(-1.0);
    }

    public void intakeOut(){
        headMotor.setPower(1.0);
    }

    public void intakeOff(){
        headMotor.setPower(0);
    }

    public void runExtension(double power){
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double finalPower = limitPower(power, minExt, minSlowExt, maxExt, maxSlowExt, extensionEncoder);
//        telemetry.addData("finalPower", finalPower);
        extensionMotor.setPower(finalPower);
    }

    public boolean safeToDump(){
        return extensionMotor.getCurrentPosition() > safeExt;
    }

    public void extensionPositionControl(int position) {
        extensionMotor.setTargetPosition(position);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setPower(0.8);
    }

    public void holdExtension() {
//        int pos = extensionMotor.getCurrentPosition();
//        extensionPositionControl(pos);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setPower(0);
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

    public RevColor.Samples getSampleColor(){
        RevColor.Samples sampleColor = revColor.checkColor();
        telemetry.addData("Color", sampleColor);
        switch (sampleColor) {
            case YELLOW:
                currentLEDAction = noLight();
                break;
            case RED:
                currentLEDAction = ledSolidRed();
                break;
            case BLUE:
                currentLEDAction = ledSlowBlinkRed();
                break;
            case NONE:
                currentLEDAction = noLight();
                break;
        }
        return sampleColor;
    }

    public void setExtensionPickup(){
        setExtPosition(pickupExtension);
    }

    public void setExtensionSafe(){
        setExtPosition(safeExt);
    }

    public void setExtensionDump(){
        setExtPosition(dumpExtension);
    }

    //Actions
    public Action grabBlock() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    headMotor.setPower(1.0);
                    initialized = true;
                }

                //TODO Read color sensor or current sensor
                return false; //Return false to end
            }
        };
    }

    public Action delayedHeadDrop(){
        return new SequentialAction(
                new SleepAction(0.5),
                new InstantAction(this::setPickup)
        );
    }

    public Action deliverToDump(){
        return new SequentialAction(
                new InstantAction(this::setExtensionSafe),
                new SleepAction(0.5),
                new InstantAction(this::setDump),
                new SleepAction(0.5),
                new InstantAction(this::setExtensionDump),
                new SleepAction(0.5),
                new InstantAction(this::intakeOut),
                new SleepAction(0.5),
                new InstantAction(this::setExtensionSafe),
                new SleepAction(0.25),
                new InstantAction(this::setPackaged)
        );
    }

    public void ledAllOff(){
        topLED_red.off();
        topLED_green.off();
    }

    //Actions
    public Action ledSolidRed() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    topLED_red.on();
                    topLED_green.off();
                    ledStop = false;
                }

                return !ledStop; //Return false to end
            }
        };
    }

    public Action ledSlowBlinkRed() {
        return new Action() {
            private boolean initialized = false;
            double lastTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    topLED_red.on();
                    topLED_green.off();
                    blinkTimer = new ElapsedTime();
                    blinkTimer.reset();
                    lastTime = blinkTimer.seconds();
                    ledStop = false;
                }
                if(ledStop){
                    ledAllOff();
                } else if(blinkTimer.seconds() - lastTime > 1){
                    if(topLED_red.isLightOn()){
                        topLED_red.off();
                    } else {
                        topLED_red.on();
                    }
                    lastTime = blinkTimer.seconds();
                }

                return !ledStop; //Return false to end
            }
        };
    }

    public Action noLight() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    ledAllOff();
                    ledStop = true;
                }

                return !ledStop; //Return false to end
            }
        };
    }



}
