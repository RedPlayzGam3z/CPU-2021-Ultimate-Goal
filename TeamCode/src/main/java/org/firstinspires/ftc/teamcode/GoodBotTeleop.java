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
// This is a test.

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
//Hiii
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy to Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="GoodBot Teleop", group="Pushbot")
//@Disabled
public class GoodBotTeleop extends LinearOpMode {

    /* Declare OpMode members. */
    GoodBotHardware robot           = new GoodBotHardware();   // Use a Pushbot's hardware
    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servodh
    public void mecanum_movement_old(double x_power, double y_power, double z_power) {
        /*
        This block calculates the power needed at each wheel. An explanation for how it works can be
        found here: https://www.youtube.com/watch?v=v7CujEW0wgc
        */
        double leftFrontPower = y_power - z_power - x_power;
        double leftRearPower = y_power - z_power + x_power;
        double rightFrontPower = y_power + z_power + x_power;
        double rightRearPower = y_power + z_power - x_power;        // Send calculated power to wheels
        robot.leftFront.setPower(leftFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightRear.setPower(rightRearPower);
    }

    public void mecanum_movement_2020(double forward, double turn, double strafe) {
        double leftFrontPower = -forward - turn - strafe;
        double leftRearPower = -forward - turn + strafe;
        double rightFrontPower = forward - turn - strafe;
        double rightRearPower = -forward + turn - strafe;
        robot.leftFront.setPower(leftFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightRear.setPower(rightRearPower);


        telemetry.addData("Right Front Power:", rightFrontPower);
        telemetry.addData("Left Front Power:", leftFrontPower);
        telemetry.addData("Right Rear Power:", rightRearPower);
        telemetry.addData("Left Rear Power:", leftRearPower);
    }

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


            mecanum_movement_2020(-gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);

            if(gamepad1.left_bumper)
            {
                robot.clawUp.setTargetPosition(829);
                robot.clawUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.clawUp.setPower(.05);
                    while (opModeIsActive() && robot.clawUp.isBusy())
                    {
                        telemetry.addData("Claw Encoder", robot.clawUp.getCurrentPosition() + "  busy=" + robot.clawUp.isBusy());
                        telemetry.update();
                    }
                robot.clawUp.setPower(0);
            }
            else if(gamepad1.right_bumper)
            {
                robot.clawUp.setTargetPosition(0);
                robot.clawUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.clawUp.setPower(-.05);
                    while (opModeIsActive() && robot.clawUp.isBusy())
                    {
                        telemetry.addData("encoder at: ", robot.clawUp.getCurrentPosition() + "  busy=" + robot.clawUp.isBusy());
                        telemetry.update();
                    }
                robot.clawUp.setPower(0);
            }


            if (gamepad1.right_trigger > 0)
                robot.clawGrip.setPower(gamepad1.right_trigger);
            else if (gamepad1.left_trigger > 0)
                robot.clawGrip.setPower(-gamepad1.left_trigger);
            else
                robot.clawGrip.setPower(0);



            telemetry.addData("Claw Vertical Power: ", robot.clawUp.getPower());
            telemetry.addData("Claw Grip Power: ", robot.clawGrip.getPower());
            telemetry.update();

            // Normalize the values so neither exceed +/- 1.0
            /*max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }
            */


           /* // Send telemetry message to signify robot running;
            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();*/

            // Pace this loop so jaw action is reasonable speed.
            //sleep(50);
        }
    }

}
