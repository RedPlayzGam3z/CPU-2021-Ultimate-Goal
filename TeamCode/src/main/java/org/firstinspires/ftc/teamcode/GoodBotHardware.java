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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class GoodBotHardware
{
    /* Public OpMode members. */
    //Drive Motors

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public DcMotor  leftFront   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  leftRear    = null;
    public DcMotor  rightRear   = null;

    //Lifty Boi
    public DcMotor  leftUp      = null;
    public DcMotor  rightUp     = null;
    //public DcMotor  dropBoi   = null;
    public Servo dropBoi        = null;
    public TouchSensor noBreak  = null;
    public Servo wobbleGrip    = null;
    public CRServo wobbleUp   = null;

    //Wobble Mover
    public CRServo wobbleUp = null;
    public Servo wobbleGrip = null;

    /* local OpMode members. */
    HardwareMap hwMap           = null;
    private ElapsedTime runTime = new ElapsedTime();

//    public void EncoderDrive(double speed,          //Speed of the motors
//                             double rightInches,    //Distance for right motors to move
//                             double leftInches,     //Distance for left motors to move
//                             double timeoutSec){    //Force function timeout
//        int leftFrontTarget;
//        int rightFrontTarget;
//        int leftRearTarget;
//        int rightRearTarget;
//
//        //Determine
//        leftFrontTarget = leftFront.getCurrentPosition() + (int)(leftInches*COUNTS_PER_INCH);
//        leftRearTarget = leftRear.getCurrentPosition() + (int)(leftInches*COUNTS_PER_INCH);
//        rightFrontTarget = rightFront.getCurrentPosition() + (int)(rightInches*COUNTS_PER_INCH);
//        rightRearTarget = rightRear.getCurrentPosition() + (int)(rightInches*COUNTS_PER_INCH);
//
//        leftFront.setTargetPosition(leftFrontTarget);
//        leftRear.setTargetPosition(leftRearTarget);
//        rightFront.setTargetPosition(rightFrontTarget);
//        rightRear.setTargetPosition(rightRearTarget);
//
//        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        runTime.reset();
//        leftFront.setPower(speed);
//        leftRear.setPower(speed);
//        rightFront.setPower(speed);
//        rightRear.setPower(speed);
//
//        while (runTime.seconds() < timeoutSec && leftFront.isBusy() && leftRear.isBusy() &&
//                rightFront.isBusy() && rightRear.isBusy()) {
//        }
//
//    }

    /* Constructor */
    public GoodBotHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront   = hwMap.get(DcMotor.class, "left_front");
        rightFront  = hwMap.get(DcMotor.class, "right_front");
        rightRear   = hwMap.get(DcMotor.class, "right_rear");
        leftRear    = hwMap.get(DcMotor.class, "left_rear");
        leftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        //clawUp      = hwMap.get(DcMotor.class, "clawUp");
        //yeet        = hwMap.get(DcMotor.class, "yeet");
        //clawUp.setDirection(DcMotor.Direction.FORWARD);
        //yeet.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
        //clawUp.setPower(0);
        //yeet.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //clawUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //yeet.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftUp      = hwMap.get(DcMotor.class, "left_lift");
        rightUp     = hwMap.get(DcMotor.class, "right_lift");
        //dropBoi     = hwMap.get(DcMotor.class, "lift_end");
        leftUp.setDirection(DcMotor.Direction.FORWARD);
        rightUp.setDirection(DcMotor.Direction.REVERSE);
        //dropBoi.setDirection(DcMotor.Direction.FORWARD);
        noBreak     = hwMap.get(TouchSensor.class, "button");

        rightUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //dropBoi.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.

        dropBoi = hwMap.get(Servo.class, "lift_end");
        dropBoi.setPosition(1);
        wobbleGrip = hwMap.get(Servo.class, "wobble_grip");
        wobbleGrip.setPosition(0);

        wobbleUp = hwMap.get(CRServo.class, "wobble_up");


        //clawGrip    = hwMap.get(CRServo.class, "clawGrip");
        //clawGrip.setPower(0);



        while(!noBreak.isPressed()) {
            rightUp.setPower(-.40);
            leftUp.setPower(-.40);
        }
        rightUp.setPower(0);
        leftUp.setPower(0);

    }
}
