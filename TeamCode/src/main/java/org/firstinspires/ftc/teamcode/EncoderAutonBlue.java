package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.sql.Time;

@Autonomous (name= "AutonBlue", group= "LinearOpMode")
public class EncoderAutonBlue extends EncoderAuton{

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
        robot.flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.flipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart(); //set robot up with intake facing wall, in line with the middle dot.
        // Should be on the side of the alliance shipping hub closest to the carousel

        robot.flipper.setTargetPosition(robot.flipper.getCurrentPosition()+100);
        robot.flipper.setVelocity(100);
        while (robot.flipper.isBusy()) {

            telemetry.addData("status:", "flipperTarget=%d, flipperCurrent=%d",
                    robot.flipper.getTargetPosition(), robot.flipper.getCurrentPosition());
            telemetry.update();
        }
        robot.flipper.setVelocity(0);
        sleep(3000);
        //robot.cap.setPosition(HardwarePushbot.CAP_CLOSED);
        sleep(1000);
telemetry.addData("Status:", "cap closed");
telemetry.update();
        /*
        encoderDrive(timer, robot, DRIVE_VELOCITY, -5); //come off of wall backwards w/ kickstand on wall
        encoderRotate(timer, robot, TURN_SPEED, 220); //rotate 180 degrees and continue as usual

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
        sleep(1000);*/
    }
}