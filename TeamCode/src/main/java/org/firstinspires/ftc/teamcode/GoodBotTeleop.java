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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//Hiii


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
    boolean invert_lift = false;
    boolean invert_drop = false;
    int inputLimit = 0;
    double contPower;
    public void mecanum_movement_old(double x_power, double y_power, double z_power) {
        /*
        This block calculates the power needed at each wheel. An explanation for how it works can be
        found here: https://www.youtube.com/watch?v=v7CujEW0wgc
        */
        double leftFrontPower = y_power - z_power - x_power;
        double leftRearPower = y_power - z_power + x_power;
        double rightFrontPower = y_power + z_power + x_power;
        double rightRearPower = y_power + z_power - x_power;
        robot.leftFront.setPower(leftFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightRear.setPower(rightRearPower);
    }

    public void mecanum_movement_2020(double forward, double turn, double strafe) {
        double leftFrontPower = forward + turn + strafe;
        double leftRearPower = forward + turn - strafe;
        double rightFrontPower = forward - turn - strafe;
        double rightRearPower = forward - turn + strafe;
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

            if (gamepad2.x && !invert_lift && inputLimit == 0) {     //Allows for toggling of inverted controls for the
                invert_lift = true;                                                //  main arm
                inputLimit = 20;
            }
            else if (gamepad2.x && invert_lift && inputLimit == 0) {
                invert_lift = false;
                inputLimit = 20;
            }

            if (gamepad2.y && !invert_drop && inputLimit == 0) {     //Allows for toggling of inverted controls for the
                invert_drop = true;                                                  //    lift end
                inputLimit = 20;
            }
            else if (gamepad2.y && invert_drop && inputLimit == 0) {
                invert_drop = false;
                inputLimit = 20;
            }

            if (inputLimit > 0)
                inputLimit -= 1;


            if (gamepad2.left_bumper && robot.noBreak.isPressed() && (gamepad2.left_stick_y >= 0) && !invert_lift) {
                robot.rightUp.setPower(gamepad2.left_stick_y*.50);
                robot.leftUp.setPower(gamepad2.left_stick_y*.50);
            }   // Above and below are the non-inverted, boosted controls of the main arm
            else if (gamepad2.left_bumper && !robot.noBreak.isPressed() && !invert_lift) {
                robot.rightUp.setPower(gamepad2.left_stick_y*.50);
                robot.leftUp.setPower(gamepad2.left_stick_y*.50);
            }

            else if (gamepad2.left_bumper && robot.noBreak.isPressed() && (gamepad2.left_stick_y <= 0) && invert_lift) {
                robot.rightUp.setPower(-gamepad2.left_stick_y*.50);
                robot.leftUp.setPower(-gamepad2.left_stick_y*.50);
            }   // Above and below are the inverted, boosted controls of the main arm
            else if (gamepad2.left_bumper && !robot.noBreak.isPressed() && invert_lift) {
                robot.rightUp.setPower(-gamepad2.left_stick_y*.50);
                robot.leftUp.setPower(-gamepad2.left_stick_y*.50);
            }

            else if(robot.noBreak.isPressed() && (gamepad2.left_stick_y >= 0) && !invert_lift) {
                robot.rightUp.setPower(gamepad2.left_stick_y * .33);
                robot.leftUp.setPower(gamepad2.left_stick_y * .33);
            }   //Above and below are the non-inverted, non-boosted controls of the main arm
            else if (!robot.noBreak.isPressed() && !invert_lift) {
                robot.rightUp.setPower(gamepad2.left_stick_y*.33);
                robot.leftUp.setPower(gamepad2.left_stick_y*.33);
            }

            else if(robot.noBreak.isPressed() && (gamepad2.left_stick_y <= 0) && invert_lift) {
                robot.rightUp.setPower(-gamepad2.left_stick_y * .33);
                robot.leftUp.setPower(-gamepad2.left_stick_y * .33);
            }   // Above and below are the inverted, non-boosted controls of the main arm
            else if (!robot.noBreak.isPressed() && invert_lift) {
                robot.rightUp.setPower(-gamepad2.left_stick_y*.33);
                robot.leftUp.setPower(-gamepad2.left_stick_y*.33);
            }

            /**
             * This section is for if dropBoi is set to be a motor
             *
             * Bellow that, the ladder structure for dropBoi to be a servo, allowing for more precise control
             */
//            if (gamepad2.right_bumper && !invert_drop){ //Boosted, non-inverted control of the lift end
//                robot.dropBoi.setPower(-gamepad2.right_stick_y*.50);
//            }
//            else if (!invert_drop) {    //Non-boosted, non-inverted control of the lift end
//                robot.dropBoi.setPower(-gamepad2.right_stick_y * .20);
//            }
//            else if (gamepad2.right_bumper && invert_drop){ //Boosted, inverted controls of the lift end
//                robot.dropBoi.setPower(gamepad2.right_stick_y*.50);
//            }
//            else if (invert_drop) {     //Non-boosted, inverted controls of the lift end
//                robot.dropBoi.setPower(gamepad2.right_stick_y * .20);
//            }

            if (robot.dropBoi.getPosition() < 1 && gamepad2.right_stick_y > 0 && !invert_drop)
                robot.dropBoi.setPosition(robot.dropBoi.getPosition()+.005);
            else if (robot.dropBoi.getPosition() > 0 && gamepad2.right_stick_y < 0 && !invert_drop)
                robot.dropBoi.setPosition(robot.dropBoi.getPosition()-.005);
            else if (robot.dropBoi.getPosition() > 0 && gamepad2.right_stick_y > 0 && invert_drop)
                robot.dropBoi.setPosition(robot.dropBoi.getPosition()-.005);
            else if (robot.dropBoi.getPosition() < 1 && gamepad2.right_stick_y < 0 && invert_drop)
                robot.dropBoi.setPosition(robot.dropBoi.getPosition()+.005);


//            if (gamepad2.dpad_down)
//                contPower = 0;
//            else if (gamepad2.dpad_up)
//                contPower = 1;
//            else
//                contPower = 0;
//
//            robot.wobbleUp.setPower(contPower);

            contPower = (gamepad2.right_trigger*2) - 1;


            robot.wobbleUp.setPower(contPower);

            if (gamepad2.dpad_left)
                robot.wobbleGrip.setPosition(1);
            else if (gamepad2.dpad_right)
                robot.wobbleGrip.setPosition(0);



            telemetry.addData("Right Trigger: ", gamepad2.right_trigger);
            telemetry.addData("Left Trigger: ", gamepad2.left_trigger);
            telemetry.addData("Wobble Up Position : ", robot.wobbleUp.getPower());
            telemetry.addData("Counter at: ", inputLimit);
            telemetry.addData("Right Up Power: ", robot.rightUp.getPower());
            telemetry.addData("Left Up Power: ", robot.leftUp.getPower());
            //telemetry.addData("Drop Power: ", robot.dropBoi.getPower());
            telemetry.addData("Drop Position: ", robot.dropBoi.getPosition());
            telemetry.addData("Button is pressed: ", robot.noBreak.isPressed());
            telemetry.addData("Invert Lift: ", invert_lift);
            telemetry.addData("Invert Drop: ", invert_drop);
            telemetry.update();

            if (gamepad2.left_stick_button)
                robot.lights.setPosition(0.35);
            else if (gamepad2.right_stick_button)
                robot.lights.setPosition(0.31);
            if (gamepad2.left_bumper)
                robot.lights.setPosition(0.57);
            else if (gamepad2.right_bumper)
                robot.lights.setPosition((0.21));






//            if(gamepad1.left_bumper)
//            {
//                robot.clawUp.setTargetPosition(500);
//                robot.clawUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.clawUp.setPower(.05);
//                    while (opModeIsActive() && robot.clawUp.isBusy())
//                    {
//                        telemetry.addData("Claw Encoder", robot.clawUp.getCurrentPosition() + "  busy=" + robot.clawUp.isBusy());
//                        telemetry.update();
//                    }
//                robot.clawUp.setPower(0);
//            }
//            else if(gamepad1.right_bumper)
//            {
//                robot.clawUp.setTargetPosition(50);
//                robot.clawUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.clawUp.setPower(-.05);
//                    while (opModeIsActive() && robot.clawUp.isBusy())
//                    {
//                        telemetry.addData("encoder at: ", robot.clawUp.getCurrentPosition() + "  busy=" + robot.clawUp.isBusy());
//                        telemetry.update();
//                    }
//                robot.clawUp.setPower(0);
//            }
//
//
//            if (gamepad2.right_bumper)  //This should open/close the claw, currently does not work
//                robot.clawGrip.setPower(1);//                          need to find out why
//            else if (gamepad2.left_bumper)
//                robot.clawGrip.setPower(-1);
//            else
//                robot.clawGrip.setPower(0)
//
//            telemetry.addData("Claw Vertical Power: ", robot.clawUp.getPower());
//            telemetry.addData("Claw Grip Power: ", robot.clawGrip.getPower());
//            telemetry.update();
        }
    }

}
