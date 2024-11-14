/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.classes.ButtonState;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

/**
 * This mode is used to tune servo positions
 */
//@Disabled
@TeleOp(group = "Test")
public class Servo_Test extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Arm
        Lift intake = new Lift(hardwareMap, telemetry);
        Servo servo = intake.flipper;
        servo.setPosition(0.5);
        int servoSelect = 0;
        int lastServoSelect = 0;

        ButtonState posInc = new ButtonState(gamepad1, ButtonState.Button.dpad_up);
        ButtonState posDec = new ButtonState(gamepad1, ButtonState.Button.dpad_down);
        ButtonState selectDown = new ButtonState(gamepad1, ButtonState.Button.dpad_left);
        ButtonState selectUp = new ButtonState(gamepad1, ButtonState.Button.dpad_right);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("Press d-pad Left/right to select a servo.");
            telemetry.addLine("On selecting a servo, it will default to pos = 0.5.");
            telemetry.addLine("Press d-pad Up/Down to move servo by 0.05");
            telemetry.addLine();
            telemetry.addLine("Current Servo: " + servo.getPortNumber());
            telemetry.addLine("Current Position: " + servo.getPosition());

            if (selectUp.newPress()) {
                if (servoSelect == 2) {
                    servoSelect = 0;
                } else {
                    servoSelect++;
                }
            }
            if (selectDown.newPress()){
                if(servoSelect ==0){
                    servoSelect = 2;}
                else{
                    servoSelect --;
                }
            }
            if(posInc.newPress()){
                servo.setPosition(servo.getPosition() + 0.05);
            }
            if(posDec.newPress()){
                servo.setPosition(servo.getPosition() - 0.05);
            }

            switch(servoSelect){
                case 0:
                    servo = intake.flipper;
                    break;
                case 1:
                    servo = intake.flipper;
                    break;
                case 2:
                    servo = intake.flipper;
                    break;
            }

            if(lastServoSelect != servoSelect) {
                servo.setPosition(0.5);
                lastServoSelect = servoSelect;}

            telemetry.update();

        }
    }
}
