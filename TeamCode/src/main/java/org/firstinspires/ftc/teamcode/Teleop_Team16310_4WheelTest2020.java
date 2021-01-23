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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a pushbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Ultimate_Game V1", group = "Test2020")
//@Disabled
public class Teleop_Team16310_4WheelTest2020 extends LinearOpMode {


    boolean rampUp = true;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor BackLeftDrive = null;
    private DcMotor BackRightDrive = null;
    private DcMotor FrontLeftDrive = null;
    private DcMotor FrontRightDrive = null;
    private DcMotor InTakeMotor = null;
    private DcMotor ConveyorMotor = null;
    private DcMotor LeftShooter = null;
    private DcMotor RightShooter = null;
    Servo   gateServo;
    Servo   armServo;
    Servo   gripServo;
    double  armPosition, gripPosition, gatePosition;
    double  MIN_POSITION = 0, MAX_POSITION = 1;


    @Override
    public void runOpMode() {

        // define the motors
        BackLeftDrive  = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRightDrive = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeftDrive  = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        InTakeMotor = hardwareMap.get(DcMotor.class, "InTake");
        ConveyorMotor = hardwareMap.get(DcMotor.class, "Conveyor");
        LeftShooter = hardwareMap.get(DcMotor.class, "LeftShooter");
        RightShooter = hardwareMap.get(DcMotor.class, "RightShooter");
        armServo = hardwareMap.get(Servo.class,"arm");
        gripServo = hardwareMap.get(Servo.class,"grip");
        // wait for start button.
        armPosition=0.09;
        gripPosition=0.95;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //This is very important to keep left drive to forward and right drive to REVERSE
        BackLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackRightDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        InTakeMotor.setDirection(DcMotor.Direction.REVERSE);
        ConveyorMotor.setDirection(DcMotor.Direction.FORWARD);
        LeftShooter.setDirection(DcMotor.Direction.FORWARD);
        RightShooter.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();

        // Scan servo till stop pressed.
        while(opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            //Please note it is very important to note that DRIVE is set to positive and

            double drive2 = gamepad1.left_stick_y;
            double turn2 = gamepad1.right_stick_x;

            // Display the current value
            telemetry.addData("game pad2 left stick Position", "%5.2f", drive2);
            telemetry.addData("game pad2 right stick Position", "%5.2f", turn2);
            telemetry.update();

            double drive = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            leftPower = Range.clip(drive * 0.7 + turn * 0.4, -1, 1);
            rightPower = Range.clip(drive * 0.7 - turn * 0.4, -1, 1);

            // Send calculated power to wheels
            BackLeftDrive.setPower(leftPower);
            BackRightDrive.setPower(rightPower);
            FrontLeftDrive.setPower(leftPower);
            FrontRightDrive.setPower(rightPower);

            if (gamepad1.left_trigger > 0) {
                InTakeMotor.setPower(gamepad1.left_trigger);
            }
            if (gamepad1.right_trigger > 0) {
                InTakeMotor.setPower(-gamepad1.right_trigger);
            }

            while (gamepad1.x) {
                InTakeMotor.setPower(1);
                ConveyorMotor.setPower(1);
                LeftShooter.setPower(1);
                RightShooter.setPower(1);
            }
            if (gamepad1.y) {
                InTakeMotor.setPower(0);
                ConveyorMotor.setPower(0);
                LeftShooter.setPower(0);
                RightShooter.setPower(0);
            }

            if (gamepad1.b) {
                //extend arm and open gripper to pick up the wobble
                armPosition = 0.22;
                gripPosition = 0.55;
                armServo.setPosition(armPosition);
                sleep(1500);
                gripServo.setPosition(gripPosition);
                ;

            }
            if (gamepad1.a) {
                //reset the arm to starting lift up position to lift the wobble
                armPosition = 0.07;
                gripPosition = 1;

                gripServo.setPosition(gripPosition);
                sleep(1000);
                armServo.setPosition(armPosition);
                armServo.close();

            }
        }  // End of While loop
    }   // end of runopmode
} // end of program


