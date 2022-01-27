package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name = "AutonTest", group= "LinearOpMode")
public class EncoderTest extends EncoderAuton{

        private ElapsedTime timer = new ElapsedTime();
        private HardwarePushbot robot = new HardwarePushbot();

        @Override
        public void runOpMode()
        {
            robot.init(hardwareMap);
            robot.blDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.flDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.frDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            robot.brDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            robot.blDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.flDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.frDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.brDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            waitForStart(); //set robot up with intake facing wall, in line with the middle dot.
            // Should be on the side of the alliance shipping hub closest to the carousel

            encoderDrive(timer, robot, DRIVE_VELOCITY, -5);
            encoderRotate(timer, robot, TURN_SPEED, 180+55);
            robot.arm.setPosition(HardwarePushbot.ARM_OUT);
            sleep(1000);
            timer.reset();
            robot.cascade.setPower(0.5);
            while (timer.milliseconds()<1200)
            { telemetry.addData("Status:", "extending cascade");
                telemetry.update();}
            robot.cascade.setPower(0);

            robot.cap.setPosition(0.8);//CAP_CLOSED and CAP_OPEN reversed, I think. Must fix in HardwarePushbot.
            sleep(1000);


            robot.cascade.setPower(-0.7);
            timer.reset();
            while (timer.milliseconds()<1000)
            { telemetry.addData("Status:", "retracting cascade");
                telemetry.update();}
            robot.cascade.setPower(0);


            robot.arm.setPosition(HardwarePushbot.ARM_IN);
            sleep(1000);

            encoderRotate(timer, robot, TURN_SPEED, 145);
            robot.blDrive.setPower(DRIVE_VELOCITY);
            robot.brDrive.setPower(DRIVE_VELOCITY);
            robot.flDrive.setPower(DRIVE_VELOCITY);
            robot.frDrive.setPower(DRIVE_VELOCITY);
            while (robot.dSensor.getDistance(DistanceUnit.MM) >70)
            {
                telemetry.addData("Status:", "looking for cascade wall");
                telemetry.addData("BL Motor Status:", "Current Position=%d", robot.blDrive.getTargetPosition());
                telemetry.update();
            }
            robot.blDrive.setPower(0);
            robot.brDrive.setPower(0);
            robot.flDrive.setPower(0);
            robot.frDrive.setPower(0);

            stall("waiting for x to spin duck");

            timer.reset();
            robot.cascade.setPower(CAROUSEL_SPEED);
            while (timer.seconds()<2) {
                telemetry.addData("Status:", "spinning carousel");
                telemetry.update();
            }
            robot.cascade.setPower(0);

            encoderDrive(timer, robot, DRIVE_VELOCITY, -100);
            stall("waiting for x to strafe left");
            encoderStrafe(timer, robot, DRIVE_VELOCITY, -5,0);
            stall("waiting for x");
            encoderDrive(timer, robot, DRIVE_VELOCITY, -20);
        }

        public void stall(String message)
        {
            while (!gamepad1.x)
            {
                telemetry.addData("Status:", message);
                telemetry.update();
            }
        }
}
