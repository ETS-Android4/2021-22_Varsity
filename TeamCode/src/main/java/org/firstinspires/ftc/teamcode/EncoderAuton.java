package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

@Disabled
public class EncoderAuton extends LinearOpMode{

    protected static final double TICKS_PER_ROTATION = 538; //real val 537.7?
    protected static final double WHEEL_DIAMETER = 4; //real val?
    // gear reduction??
    protected static final double ROBOT_DIAMETER = 16.64; //real val?

    protected static final double DRIVE_VELOCITY = 1000;
    protected static final double TURN_SPEED = 0.4;
    protected static final double CASCADE_SPEED = 0.4; //find correct value
    protected static final double CAROUSEL_SPEED = 0.4; //find correct value

    @Override
    public void runOpMode()
    {
    }

    protected void encoderStrafe(ElapsedTime aTimer, HardwarePushbot aRobot, double speed, double xDistance, double yDistance)
    {
        int newBLTarget = aRobot.blDrive.getCurrentPosition() + (int) (TICKS_PER_ROTATION*(xDistance+yDistance) / (WHEEL_DIAMETER * Math.PI));
        int newFLTarget = aRobot.flDrive.getCurrentPosition() + (int) (TICKS_PER_ROTATION*(-xDistance+yDistance) / (WHEEL_DIAMETER * Math.PI));
        int newBRTarget = aRobot.brDrive.getCurrentPosition() - (int) (TICKS_PER_ROTATION*(-xDistance+yDistance) / (WHEEL_DIAMETER * Math.PI));
        int newFRTarget = aRobot.frDrive.getCurrentPosition() - (int) (TICKS_PER_ROTATION*(xDistance+yDistance) / (WHEEL_DIAMETER * Math.PI));

        aRobot.blDrive.setTargetPosition(newBLTarget);
        aRobot.flDrive.setTargetPosition(newFLTarget);
        aRobot.brDrive.setTargetPosition(newBRTarget);
        aRobot.frDrive.setTargetPosition(newFRTarget);

        aRobot.blDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        aRobot.brDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        aRobot.flDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        aRobot.frDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        aRobot.blDrive.setVelocity(speed);
        aRobot.brDrive.setVelocity(speed);
        aRobot.frDrive.setVelocity(speed);
        aRobot.flDrive.setVelocity(speed);

        aTimer.reset();

        while (aRobot.blDrive.isBusy() && aRobot.brDrive.isBusy() && aRobot.flDrive.isBusy() && aRobot.frDrive.isBusy())
        {
            telemetry.addData("BL Motor Status:\n", "Target Position=%d\nCurrent Position=%d\nSpeed=%f",
                    aRobot.blDrive.getTargetPosition(), aRobot.blDrive.getCurrentPosition(), aRobot.blDrive.getPower());
            telemetry.addData("BR Motor Status:\n", "Target Position=%d\nCurrent Position=%d\nSpeed=%f",
                    aRobot.brDrive.getTargetPosition(), aRobot.brDrive.getCurrentPosition(), aRobot.brDrive.getPower());
            telemetry.addData("FL Motor Status:\n", "Target Position=%d\nCurrent Position=%d\nSpeed=%f",
                    aRobot.flDrive.getTargetPosition(), aRobot.flDrive.getCurrentPosition(), aRobot.flDrive.getPower());
            telemetry.addData("FR Motor Status:\n", "Target Position=%d\nCurrent Position=%d\nSpeed=%f",
                    aRobot.frDrive.getTargetPosition(), aRobot.frDrive.getCurrentPosition(), aRobot.frDrive.getPower());
            telemetry.update();
        }
    }

    protected void encoderRotate(ElapsedTime timer, HardwarePushbot aRobot, double speed, double angle) {
        if (opModeIsActive()) {
            double arcLength = (angle/360)*(ROBOT_DIAMETER*Math.PI);
            int newBLTarget = aRobot.blDrive.getCurrentPosition() + (int)(TICKS_PER_ROTATION *arcLength/(WHEEL_DIAMETER*Math.PI));
            int newFLTarget = aRobot.flDrive.getCurrentPosition() + (int)(TICKS_PER_ROTATION*arcLength/(WHEEL_DIAMETER*Math.PI));
            int newBRTarget = aRobot.brDrive.getCurrentPosition() - (int)(TICKS_PER_ROTATION*arcLength/(WHEEL_DIAMETER*Math.PI));
            int newFRTarget = aRobot.frDrive.getCurrentPosition() - (int)(TICKS_PER_ROTATION*arcLength/(WHEEL_DIAMETER*Math.PI));


            aRobot.blDrive.setTargetPosition(newBLTarget);
            aRobot.flDrive.setTargetPosition(newFLTarget);
            aRobot.brDrive.setTargetPosition(newBRTarget);
            aRobot.frDrive.setTargetPosition(newFRTarget);

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
                    && aRobot.flDrive.isBusy() && aRobot.brDrive.isBusy() && aRobot.frDrive.isBusy())
            {
                telemetry.addData("Completing Action. Time taken:", timer.seconds());
                telemetry.update();
            }


            aRobot.blDrive.setPower(0);
            aRobot.brDrive.setPower(0);
            aRobot.frDrive.setPower(0);
            aRobot.flDrive.setPower(0);
        }
    }


    protected void encoderCascadeExtend(ElapsedTime timer, HardwarePushbot aRobot, double speed, double proportionExtended) //"proportionExtended" means if halfway extended, write 0.5
    {
        if (opModeIsActive())
        {
            int newCascadeTarget = aRobot.cascade.getCurrentPosition() + (int)(proportionExtended*6.25*TICKS_PER_ROTATION);
            // found that 6.25 rotations brings out the cascade: https://www.gobilda.com/low-side-cascading-kit-four-stage-752mm-travel/
            aRobot.cascade.setTargetPosition(newCascadeTarget);
            aRobot.cascade.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            aRobot.cascade.setPower(speed);

            timer.reset();

            while (opModeIsActive() && aRobot.cascade.isBusy())
            {
                telemetry.addData("Completing Action. Time taken:", timer.seconds());
                telemetry.update();
            }

            aRobot.cascade.setPower(0);
        }
    }

//don't need because cascadeExtend does the same thing
/*    protected void encoderCascadeRetract(double speed, double proportionRetracted)
    {
        if (opModeIsActive())
        {
            int newCascadeTarget = aRobot.cascade.getCurrentPosition() - (int)(proportionRetracted*6.25*TICKS_PER_ROTATION);

            aRobot.cascade.setTargetPosition(newCascadeTarget);
            aRobot.cascade.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            aRobot.cascade.setPower(-speed);

            timer.reset();

            while (opModeIsActive() && aRobot.cascade.isBusy())
            {
                telemetry.addData("Completing Action. Time taken:", timer.seconds());
                telemetry.update();
            }

            aRobot.cascade.setPower(0);
        }
    }*/


    protected void encoderDrive(ElapsedTime aTimer, HardwarePushbot aRobot, double speed, double distance)
    {
        if (opModeIsActive()) {
            int newBLTarget = aRobot.blDrive.getCurrentPosition() + (int) (TICKS_PER_ROTATION*distance / (WHEEL_DIAMETER * Math.PI));
            int newBRTarget = aRobot.brDrive.getCurrentPosition() + (int) (TICKS_PER_ROTATION* distance / (WHEEL_DIAMETER * Math.PI));
            int newFLTarget = aRobot.flDrive.getCurrentPosition() + (int) (TICKS_PER_ROTATION* distance / (WHEEL_DIAMETER * Math.PI));
            int newFRTarget = aRobot.frDrive.getCurrentPosition() + (int) (TICKS_PER_ROTATION*distance / (WHEEL_DIAMETER * Math.PI));

            aRobot.blDrive.setTargetPosition(newBLTarget);
            aRobot.flDrive.setTargetPosition(newFLTarget);
            aRobot.brDrive.setTargetPosition(newBRTarget);
            aRobot.frDrive.setTargetPosition(newFRTarget);

            aRobot.blDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            aRobot.brDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            aRobot.flDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            aRobot.frDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            aRobot.blDrive.setVelocity(speed);
            aRobot.brDrive.setVelocity(speed);
            aRobot.frDrive.setVelocity(speed);
            aRobot.flDrive.setVelocity(speed);

            aTimer.reset();

            while (aRobot.blDrive.isBusy() && aRobot.brDrive.isBusy() && aRobot.flDrive.isBusy() && aRobot.frDrive.isBusy())
            {
                telemetry.addData("BL Motor Status:\n", "Target Position=%d\nCurrent Position=%d\nSpeed=%f",
                        aRobot.blDrive.getTargetPosition(), aRobot.blDrive.getCurrentPosition(), aRobot.blDrive.getPower());
                telemetry.addData("BR Motor Status:\n", "Target Position=%d\nCurrent Position=%d\nSpeed=%f",
                        aRobot.brDrive.getTargetPosition(), aRobot.brDrive.getCurrentPosition(), aRobot.brDrive.getPower());
                telemetry.addData("FL Motor Status:\n", "Target Position=%d\nCurrent Position=%d\nSpeed=%f",
                        aRobot.flDrive.getTargetPosition(), aRobot.flDrive.getCurrentPosition(), aRobot.flDrive.getPower());
                telemetry.addData("FR Motor Status:\n", "Target Position=%d\nCurrent Position=%d\nSpeed=%f",
                        aRobot.frDrive.getTargetPosition(), aRobot.frDrive.getCurrentPosition(), aRobot.frDrive.getPower());
                telemetry.update();
            }
        }
    }
}