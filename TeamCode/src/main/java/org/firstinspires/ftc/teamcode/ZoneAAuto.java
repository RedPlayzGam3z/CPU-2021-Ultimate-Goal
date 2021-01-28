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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Sherri Auto, Zone A", group="Pushbot")
//@Disabled
public class ZoneAAuto extends LinearOpMode {

    /* Declare OpMode members. */
    GoodBotHardware robot   = new GoodBotHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    public void mecanum_movement_2020(double forward,   //1 to go forward, -1 to go backwards
                                      double turn,      //1 to turn right, -1 to turn left
                                      double strafe) {  //1 to strafe right, -1 to strafe left
        double leftFrontPower = forward + turn + strafe;
        double leftRearPower = forward + turn - strafe;
        double rightFrontPower = forward - turn - strafe;
        double rightRearPower = forward - turn + strafe;
        robot.leftFront.setPower(leftFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightRear.setPower(rightRearPower);
    }

    public void EncoderDrive(double speed,          //Speed of the motors
                             double rightInches,    //Distance for right motors to move
                             double leftInches){     //Distance for left motors to move

        int leftFrontTarget;
        int rightFrontTarget;
        int leftRearTarget;
        int rightRearTarget;

        //Determine
        leftFrontTarget = robot.leftFront.getCurrentPosition() +
                (int)(leftInches* GoodBotHardware.COUNTS_PER_INCH);
        leftRearTarget = robot.leftRear.getCurrentPosition() +
                (int)(leftInches* GoodBotHardware.COUNTS_PER_INCH);
        rightFrontTarget = robot.rightFront.getCurrentPosition() +
                (int)(rightInches* GoodBotHardware.COUNTS_PER_INCH);
        rightRearTarget = robot.rightRear.getCurrentPosition() +
                (int)(rightInches* GoodBotHardware.COUNTS_PER_INCH);

        robot.leftFront.setTargetPosition(leftFrontTarget);
        robot.leftRear.setTargetPosition(leftRearTarget);
        robot.rightFront.setTargetPosition(rightFrontTarget);
        robot.rightRear.setTargetPosition(rightRearTarget);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        robot.leftFront.setPower(speed);
        robot.leftRear.setPower(speed);
        robot.rightFront.setPower(speed);
        robot.rightRear.setPower(speed);

        while (opModeIsActive()  && (robot.leftFront.isBusy()  || robot.leftRear.isBusy() ||
                robot.rightFront.isBusy() || robot.rightRear.isBusy())) {
            telemetry.addData("RF Position: ", robot.rightFront.getCurrentPosition());
            telemetry.addData("RF Target: ", robot.rightFront.getTargetPosition());
            telemetry.addData("RR Position: ", robot.rightRear.getCurrentPosition());
            telemetry.addData("RR Target: ", robot.rightRear.getTargetPosition());
            telemetry.addData("LF Position: ", robot.leftFront.getCurrentPosition());
            telemetry.addData("LF Target: ", robot.leftFront.getTargetPosition());
            telemetry.addData("LR Position: ", robot.leftRear.getCurrentPosition());
            telemetry.addData("LR Target: ", robot.leftRear.getTargetPosition());
            telemetry.update();
        }

        robot.leftFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightRear.setPower(0);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        //RIP Hokey Poky, Shall be missed
//        mecanum_movement_2020(1,0,0);   //Forward 1 second
//        sleep(3000);
//
//        mecanum_movement_2020(0,-1,0);
//        sleep(500);
//
//        mecanum_movement_2020(1,0,0);
//        sleep(500);
//
//        mecanum_movement_2020(-1,0,0);
//        sleep(500);
//
//        mecanum_movement_2020(0,1,0);
//        sleep(500);
//
//        mecanum_movement_2020(-1,0,0);
//        sleep(1000);
//
//        mecanum_movement_2020(0,1,0);
//        sleep(500);
//
//        mecanum_movement_2020(1,0,0);
//        sleep(2000);
//
//        mecanum_movement_2020(0,0,0);


        EncoderDrive(1, 70,70); //Forward
        EncoderDrive(.25, 10, -10); //Turn to face zone A
        EncoderDrive(1,5,5); //Move into zone A
        EncoderDrive(1, -5,-5); //Move back onto the line
        EncoderDrive(1, -20,-20); // Move back for hokey poky
        EncoderDrive(1, -100,100); //Spin for hokey poky
        EncoderDrive(1, 20,20); // Park
    }

}
