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
import com.qualcomm.robotcore.util.Hardware;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@TeleOp(name="Devbot Teleop", group="Pushbot")
@Disabled
public class TeleOpExample extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareExamples robot  = new HardwareExamples();   // Use a Pushbot's hardware


    //Allows for mecanum movement of the robot, omnidirectional movement
        //Commented out so GIT doesn't scream about it
//    public void mecanum_movement_2020(double forward, double turn, double strafe) {
//        double leftFrontPower = forward + turn + strafe;
//        double leftRearPower = forward + turn - strafe;
//        double rightFrontPower = forward - turn - strafe;
//        double rightRearPower = forward - turn + strafe;
//        robot.leftFront.setPower(leftFrontPower);
//        robot.leftRear.setPower(leftRearPower);
//        robot.rightFront.setPower(rightFrontPower);
//        robot.rightRear.setPower(rightRearPower);
//
//        telemetry.addData("Right Front Power:", rightFrontPower);
//        telemetry.addData("Left Front Power:", leftFrontPower);
//        telemetry.addData("Right Rear Power:", rightRearPower);
//        telemetry.addData("Left Rear Power:", leftRearPower);
//
//    }

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            //mecanum_movement_2020(gamepad1.left_stick_y, -gamepad1.right_stick_x, -gamepad1.left_stick_x);

            robot.motorExample.setPower((gamepad1.right_trigger/2) + (gamepad1.left_trigger/2));
            //Half speed with one trigger, Full speed with both

            robot.servoExample.setPosition(1); //Servos have a range of 0-1, setting its position
                //to that value and it then holds that position until another number is sent to it

            robot.crServoExample.setPower(-1); //You would think this would make it keep moving
                //You would be wrong. CRServos may say they use setPower, but really they are
                //setting position. However unlike normal servos, they need to constintely be set to
                //that position, so use a variable that changes based on its current value to allow
                //you not to have to feed it numbers constantly.

            if (robot.buttonExample.isPressed()) { //if the button is pushed down, run this block
                //code here
            }
            else if (!robot.buttonExample.isPressed()){ //if the button is not pushed down, run this block
                //code here
            }
                    




            telemetry.update();
        }
    }
}
