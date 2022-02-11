package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.sql.Time;

@Autonomous (name= "AutonTest", group= "LinearOpMode")
public class EncoderTest extends EncoderAuton {

    private ElapsedTime timer = new ElapsedTime();
    private HardwarePushbot robot = new HardwarePushbot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.blDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.flDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.frDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.brDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        robot.flipper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.blDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.flDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.frDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.brDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flipper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        int turnDegrees=300;
///equation: undershoot =-0.35x -5.31
/*        while(!gamepad1.x)
        {
            if(gamepad1.right_bumper)
            {
                turnDegrees+=10;
            }
            else if(gamepad1.left_bumper)
            {
                turnDegrees-=10;
            }
            telemetry.addData("Status: ", "turnDegrees=%d", turnDegrees);
            telemetry.update();

        }*/
        telemetry.addData("Status: TURNING", "turnDegrees=%d", turnDegrees);
        telemetry.update();

        encoderRotate(timer, robot, TURN_SPEED, turnDegrees);

    }

    @Override
    protected void encoderRotate(ElapsedTime timer, HardwarePushbot aRobot, double speed, double angle) {
        if (opModeIsActive()) {
            double arcLength = (angle / 360) * (ROBOT_DIAMETER * Math.PI);
            int BLTarget = (int) (TICKS_PER_ROTATION * arcLength / (WHEEL_DIAMETER * Math.PI));
            int FLTarget = (int) (TICKS_PER_ROTATION * arcLength / (WHEEL_DIAMETER * Math.PI));
            int BRTarget = (int) -(TICKS_PER_ROTATION * arcLength / (WHEEL_DIAMETER * Math.PI));
            int FRTarget = (int) -(TICKS_PER_ROTATION * arcLength / (WHEEL_DIAMETER * Math.PI));


            aRobot.blDrive.setTargetPosition(BLTarget);
            aRobot.flDrive.setTargetPosition(FLTarget);
            aRobot.brDrive.setTargetPosition(BRTarget);
            aRobot.frDrive.setTargetPosition(FRTarget);

            aRobot.blDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            aRobot.brDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            aRobot.flDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            aRobot.frDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            aRobot.blDrive.setPower(speed);
            aRobot.brDrive.setPower(speed);
            aRobot.frDrive.setPower(speed);
            aRobot.flDrive.setPower(speed);

            timer.reset();

            while (opModeIsActive() && aRobot.blDrive.isBusy()
                    && aRobot.flDrive.isBusy() && aRobot.brDrive.isBusy() && aRobot.frDrive.isBusy()) {

                int ticks = (int)(robot.blDrive.getCurrentPosition());

                int degrees = (int) (360 * ticks * WHEEL_DIAMETER / (TICKS_PER_ROTATION * ROBOT_DIAMETER));

                if (ticks%20<5 &&ticks%20>0) {
                    telemetry.addData("Status", "degrees=%d, ticks=%d", ticks, degrees);
                    telemetry.update();
                }
            }


            aRobot.blDrive.setPower(0);
            aRobot.brDrive.setPower(0);
            aRobot.frDrive.setPower(0);
            aRobot.flDrive.setPower(0);
            telemetry.update();
        }
    }
}