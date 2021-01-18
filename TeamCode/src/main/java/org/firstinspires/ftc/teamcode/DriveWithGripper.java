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

// simple teleop program that drives bot using controller joysticks in tank mode.
// this code monitors the period and stops when the period is ended. Also uses
// controller button A to lower the arm, button B to raise the arm. Controller
// button X opens the gripper and button Y closes the gripper.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Drive Gripper", group="Exercises")
//@Disabled
public class DriveWithGripper extends LinearOpMode
{
    Servo   armServo, gripServo;
    CRServo contServo;
    float   leftY, rightY;
    double  armPosition, gripPosition, contPower;
    double  MIN_POSITION = 0, MAX_POSITION = 1;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {

       armServo = hardwareMap.servo.get("arm");
       gripServo = hardwareMap.servo.get("grip");


        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.
        //gripPosition=0.001;
        armServo.setPosition(0.20);


        waitForStart();
        armServo.setPosition(0.20);


        while (opModeIsActive())
        {
           telemetry.addData("Mode", "running");

            // check the gamepad buttons and if pressed, increment the appropriate position
            // variable to change the servo location.

            // move arm down on A button if not already at lowest position.
            if (gamepad1.a )  armPosition = 0.15;

            // move arm up on B button if not already at the highest position.
            if (gamepad1.b ) armPosition = 0.35;



            // set the servo position/power values as we have computed them.
           armServo.setPosition(armPosition);
         //  gripServo.setPosition(Range.clip(gripPosition, MIN_POSITION, MAX_POSITION));
          //  contServo.setPower(contPower);


            telemetry.addData("arm servo", String.format("position=%.2f  actual=%.2f", gripPosition,
                armServo.getPosition()));

        //    telemetry.addData("grip servo", "position=%.2f  actual=%.2f", gripPosition, gripServo.getPosition());

            telemetry.update();
            idle();
        }
    }
}