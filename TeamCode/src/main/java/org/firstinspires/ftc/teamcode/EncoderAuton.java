package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name= "Basic: EncoderAuton", group= "Linear OpMode")
@Disabled
public class EncoderAuton extends LinearOpMode{

    private ElapsedTime timer = new ElapsedTime();
    private HardwarePushbot robot = new HardwarePushbot();

    private static final double TICKS_PER_ROTATION = 538; //real val 537.7?
    private static final double WHEEL_DIAMETER = 3.7; //real val?
    // gear reduction??
    private static final double ROBOT_DIAMETER = 17.15; //real val?

    private static final double DRIVE_SPEED = 0.6;
    private static final double TURN_SPEED = 0.4;

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);
        robot.blDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.brDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.blDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.brDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        encoderDrive(DRIVE_SPEED, 10);
        encoderRotate(TURN_SPEED, 180);
        encoderDrive(DRIVE_SPEED, 5);
        encoderRotate(DRIVE_SPEED, 360);
    }

    private void encoderDrive(double speed, double distance)
    {
        if (opModeIsActive()) {
            int newBLTarget = robot.blDrive.getCurrentPosition() + (int) (distance / (WHEEL_DIAMETER * Math.PI * TICKS_PER_ROTATION));
            int newBRTarget = robot.brDrive.getCurrentPosition() + (int) (distance / (WHEEL_DIAMETER * Math.PI * TICKS_PER_ROTATION));
            int newFLTarget = robot.flDrive.getCurrentPosition() + (int) (distance / (WHEEL_DIAMETER * Math.PI * TICKS_PER_ROTATION));
            int newFRTarget = robot.frDrive.getCurrentPosition() + (int) (distance / (WHEEL_DIAMETER * Math.PI * TICKS_PER_ROTATION));

            robot.blDrive.setTargetPosition(newBLTarget);
            robot.flDrive.setTargetPosition(newFLTarget);
            robot.brDrive.setTargetPosition(newBRTarget);
            robot.frDrive.setTargetPosition(newFRTarget);

            robot.blDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.brDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.flDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.blDrive.setPower(speed);
            robot.brDrive.setPower(speed);
            robot.frDrive.setPower(speed);
            robot.flDrive.setPower(speed);

            timer.reset();

            while (opModeIsActive() && robot.blDrive.isBusy()
                    && robot.flDrive.isBusy() && robot.brDrive.isBusy() && robot.frDrive.isBusy())
            {
                telemetry.addData("Completing Action. Time taken:", timer.seconds());
                telemetry.update();
            }


            robot.blDrive.setPower(0);
            robot.brDrive.setPower(0);
            robot.frDrive.setPower(0);
            robot.flDrive.setPower(0);
        }
    }

    private void encoderStrafe(double speed, double distance, double angle)
    {

    }

    private void encoderRotate(double speed, double angle) {
        if (opModeIsActive()) {
            int newBLTarget = robot.blDrive.getCurrentPosition() + (int)((angle/360)*(ROBOT_DIAMETER*Math.PI));
            int newFLTarget = robot.blDrive.getCurrentPosition() + (int)((angle/360)*(ROBOT_DIAMETER*Math.PI));
            int newBRTarget = robot.blDrive.getCurrentPosition() - (int)((angle/360)*(ROBOT_DIAMETER*Math.PI));
            int newFRTarget = robot.blDrive.getCurrentPosition() - (int)((angle/360)*(ROBOT_DIAMETER*Math.PI));


            robot.blDrive.setTargetPosition(newBLTarget);
            robot.flDrive.setTargetPosition(newFLTarget);
            robot.brDrive.setTargetPosition(newBRTarget);
            robot.frDrive.setTargetPosition(newFRTarget);

            robot.blDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.brDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.flDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.blDrive.setPower(speed);
            robot.brDrive.setPower(speed);
            robot.frDrive.setPower(speed);
            robot.flDrive.setPower(speed);

            timer.reset();

            while (opModeIsActive() && robot.blDrive.isBusy()
                    && robot.flDrive.isBusy() && robot.brDrive.isBusy() && robot.frDrive.isBusy())
            {
                telemetry.addData("Completing Action. Time taken:", timer.seconds());
                telemetry.update();
            }


            robot.blDrive.setPower(0);
            robot.brDrive.setPower(0);
            robot.frDrive.setPower(0);
            robot.flDrive.setPower(0);
        }
    }

}
