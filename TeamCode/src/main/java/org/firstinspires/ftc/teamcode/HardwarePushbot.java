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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
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
public class HardwarePushbot
{
    /* Public OpMode members. */
    public DcMotorEx blDrive = null;
    public DcMotorEx brDrive = null;
    public DcMotorEx flDrive = null;
    public DcMotorEx frDrive = null;
    public DcMotorEx cascade = null;
    public DcMotorEx intake = null;
    public DcMotorEx flipper = null;
    public DcMotorEx carousel = null;

    public CRServo rotator = null;
    public Servo arm = null;
    public Servo cap = null;

    public DistanceSensor dSensor = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    //private ElapsedTime period  = new ElapsedTime();
    public static final double MAX_SPEED= 1;
    public static final double ARM_OUT= 0;
    public static final double ARM_IN = 1;
    public static final double CAP_CLOSED = 0; //find actual value
    public static final double CAP_OPEN = 1; //find actual value

    /* Constructor */
    public HardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        blDrive  = hwMap.get(DcMotorEx.class, "back_left");
        brDrive  = hwMap.get(DcMotorEx.class, "back_right");
        flDrive  = hwMap.get(DcMotorEx.class, "front_left");
        frDrive  = hwMap.get(DcMotorEx.class, "front_right");
        cascade = hwMap.get(DcMotorEx.class, "cascade");
        intake = hwMap.get(DcMotorEx.class, "intake");
        flipper = hwMap.get(DcMotorEx.class, "flipper");
        carousel = hwMap.get(DcMotorEx.class, "carousel");

        rotator = hwMap.get(CRServo.class, "rotator");
        arm = hwMap.get(Servo.class, "arm");
        cap = hwMap.get(Servo.class, "cap");

        dSensor = hwMap.get(DistanceSensor.class, "distanceSensor");

        blDrive.setDirection(DcMotorEx.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        brDrive.setDirection(DcMotorEx.Direction.FORWARD);
        flDrive.setDirection(DcMotorEx.Direction.FORWARD);
        frDrive.setDirection(DcMotorEx.Direction.FORWARD);
        cascade.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        flipper.setDirection(DcMotorEx.Direction.REVERSE);
        carousel.setDirection(DcMotorEx.Direction.FORWARD);
        rotator.setDirection(CRServo.Direction.FORWARD);

        // Set motors and CRServos to zero power
        blDrive.setPower(0);
        brDrive.setPower(0);
        flDrive.setPower(0);
        frDrive.setPower(0);
        cascade.setPower(0);
        intake.setPower(0);
        flipper.setPower(0);
        carousel.setPower(0);
        rotator.setPower(0);

        //Set servos to correct locations
        arm.setPosition(ARM_IN);
        cap.setPosition(CAP_OPEN);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        blDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        brDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        cascade.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        carousel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flipper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
 }

