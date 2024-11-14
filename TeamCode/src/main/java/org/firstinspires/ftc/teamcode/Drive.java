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

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.classes.ButtonState;
import org.firstinspires.ftc.teamcode.classes.RevColor;

import java.util.ArrayList;
import java.util.List;


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

@TeleOp(name="Drive", group="Comp")
//@Disabled
public class Drive extends LinearOpMode {

    @Override
    public void runOpMode() {
        FtcDashboard dash = FtcDashboard.getInstance();
        List<Action> runningActions = new ArrayList<>();

        TelemetryPacket packet = new TelemetryPacket();
        Robot m_robot = new Robot(hardwareMap, telemetry, new Pose2d(0,0,0));

        double powerScale=1;

        double liftPower;
        boolean liftInHold = false;
        boolean liftManualOp = true;

        PoseVelocity2d driveControl;


        ButtonState autoIntake = new ButtonState(gamepad2, ButtonState.Button.right_trigger);
        ButtonState autoBackIntake = new ButtonState(gamepad2, ButtonState.Button.right_bumper);
        ButtonState zeroLift = new ButtonState(gamepad1, ButtonState.Button.back);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("Extension Position", m_robot.intake.getCurrentExt());
            telemetry.addData("Lift Position", m_robot.lift.getCurrentExt());


            m_robot.intake.getSampleColor();

            if (gamepad1.right_bumper) {
                powerScale=1;
            } else if (gamepad1.left_bumper) {
                powerScale=.5;
            }

            Vector2d input = new Vector2d(
                    Math.pow(-gamepad1.left_stick_y, 3) * powerScale,
                    Math.pow(-gamepad1.left_stick_x, 3) * powerScale);

            double rotation = Math.pow(-gamepad1.right_stick_x, 3) * powerScale;

            driveControl = new PoseVelocity2d(input, rotation);

            m_robot.drive.setDrivePowers(driveControl);
            if (zeroLift.newPress()) {
                m_robot.lift.zeroLift();
            }

            //TODO: Add spin buttons

            //Intake Controls
            if (Math.abs(gamepad2.right_stick_y) > 0.05) {
                m_robot.intake.runExtension(-gamepad2.right_stick_y);
            } else {
                m_robot.intake.runExtension(0);
            }

            //Floor positions
            if (gamepad2.a) {
                m_robot.intake.setPickup();
            }

            if (gamepad2.y) {
                m_robot.intake.setPackaged();
            }

            if (gamepad2.b) {
                m_robot.intake.setDump();
                m_robot.intake.setDumpExt();
            }

            if (autoIntake.newPress()) {
                m_robot.intake.intakeIn();
            } else if (autoIntake.getCurrentPress()) {
                if (m_robot.intake.revColor.closestColor() != RevColor.Samples.NONE) {
                    m_robot.intake.intakeOff();
                }
            } else if (gamepad2.left_trigger > 0.05) {
                m_robot.intake.intakeOut();
            } else {
                m_robot.intake.intakeOff();
            }



            //Lift controls
            if(gamepad2.x && m_robot.intake.liftSafe()){
                m_robot.lift.setFlip();
            } else {
                m_robot.lift.setFlipperReady();
            }


            //Back Intake Control
            if (autoBackIntake.newPress()) {
                m_robot.lift.intakeIn();
            } else if (autoBackIntake.getCurrentPress()) {
                if (m_robot.lift.intakeCurrentSpike()) {
                    m_robot.lift.intakeOff();
                }
            } else if (gamepad2.left_bumper) {
                m_robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));
                m_robot.lift.autoClip(Lift.LiftPositions.HIGH_CLIP_RELEASE);
            } else if (gamepad2.back){
                m_robot.lift.intakeOut();
            }
            else {
                m_robot.lift.intakeOff();
            }

            //Lift height control
            if (m_robot.intake.liftSafe()) {
                if (gamepad2.dpad_up) {
                    m_robot.lift.elevatorPositionByConstant(Lift.LiftPositions.HIGH_CLIP);
                    liftManualOp = false;
                } else if (gamepad2.dpad_right) {
                    m_robot.lift.elevatorPositionByConstant(Lift.LiftPositions.HIGH_BUCKET);
                    liftManualOp = false;
                } else if (gamepad2.dpad_left) {
                    m_robot.lift.elevatorPositionByConstant(Lift.LiftPositions.LOW_BUCKET);
                    liftManualOp = false;
                } else if (gamepad2.dpad_down) {
                    m_robot.lift.elevatorPositionByConstant(Lift.LiftPositions.CLIP_PICKUP);
                    liftManualOp = false;
                }

                liftPower = -gamepad2.left_stick_y;
                if (Math.abs(liftPower) > 0.05) {
                    m_robot.lift.runLift(liftPower);
                    liftInHold = false;
                    liftManualOp = true;
                } else if (liftManualOp && !liftInHold) {
                    m_robot.lift.holdElevator();
                    liftInHold = true;
                    liftManualOp = false;
                }
            }

//            if(Math.abs(-gamepad2.left_stick_y) > 0.05){
//                m_robot.lift.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                m_robot.lift.liftMotor.setPower(-gamepad2.left_stick_y);
//            } else {
//                m_robot.lift.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                m_robot.lift.liftMotor.setPower(0);
//            }


            // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);

            telemetry.update();

        }
    }
}
