package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name= "Basic: EncoderAutonRed", group= "Linear OpMode")
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
    private static final double CASCADE_SPEED = 0.4; //find correct value
    private static final double CAROUSEL_SPEED = 0.4; //find correct value


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

        waitForStart(); //set robot up with intake facing wall, in line with the middle dot.
        // Should be on the side of the alliance shipping hub closest to the carousel

        encoderDrive(-DRIVE_SPEED, 24); //make sure that this moves the robot's non-intake side forward
        encoderRotate(TURN_SPEED, 45); //make sure this turns the robot clockwise

        robot.cap.setPosition(HardwarePushbot.CAP_CLOSED);
        robot.arm.setPosition(HardwarePushbot.ARM_OUT);
        encoderCascadeExtend(CASCADE_SPEED, 1);
        robot.cap.setPosition(HardwarePushbot.CAP_OPEN);
        encoderCascadeRetract(CASCADE_SPEED, 1);
        robot.arm.setPosition(HardwarePushbot.ARM_IN);

        encoderRotate(TURN_SPEED, 45); //should turn clockwise; robot intake should be facing wall with storage unit.
        encoderDrive(DRIVE_SPEED, 35);// Carousel spinning wheel should line up with carousel
        robot.carousel.setPower(CAROUSEL_SPEED);
        timer.reset();
        while (opModeIsActive() && timer.seconds()<1) //how long does it actually take to deliver a ducky?
        {
            telemetry.addData("Completing Action. Time taken:", timer.seconds());
            telemetry.update();
        }
        robot.carousel.setPower(0);


        robot.blDrive.setTargetPosition(robot.blDrive.getCurrentPosition() + (int) (22 / (WHEEL_DIAMETER * Math.PI * TICKS_PER_ROTATION)));
        robot.flDrive.setTargetPosition(robot.flDrive.getCurrentPosition() - (int) (22 / (WHEEL_DIAMETER * Math.PI * TICKS_PER_ROTATION)));
        robot.brDrive.setTargetPosition(robot.brDrive.getCurrentPosition() - (int) (22 / (WHEEL_DIAMETER * Math.PI * TICKS_PER_ROTATION)));
        robot.frDrive.setTargetPosition(robot.frDrive.getCurrentPosition() + (int) (22 / (WHEEL_DIAMETER * Math.PI * TICKS_PER_ROTATION)));

        robot.blDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.brDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.flDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.blDrive.setPower(DRIVE_SPEED);
        robot.brDrive.setPower(DRIVE_SPEED);
        robot.frDrive.setPower(DRIVE_SPEED);
        robot.flDrive.setPower(DRIVE_SPEED);

        timer.reset();
        while (opModeIsActive() && robot.blDrive.isBusy()
                && robot.flDrive.isBusy() && robot.brDrive.isBusy() && robot.frDrive.isBusy()) // wait till all motors are done
        {
            telemetry.addData("Completing Action. Time taken:", timer.seconds());
            telemetry.update();
        }
        robot.blDrive.setPower(0); //stop motors
        robot.brDrive.setPower(0);
        robot.frDrive.setPower(0);
        robot.flDrive.setPower(0);
    }

    private void encoderStrafe(double speed, double xDistance, double yDistance)
    {
        //not today!
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


    private void encoderCascadeExtend(double speed, double proportionExtended) //"proportionExtended" means if halfway extended, write 0.5
    {
        if (opModeIsActive())
        {
            int newCascadeTarget = robot.cascade.getCurrentPosition() + (int)(proportionExtended*6.25*TICKS_PER_ROTATION);
            // found that 6.25 rotations brings out the cascade: https://www.gobilda.com/low-side-cascading-kit-four-stage-752mm-travel/
            robot.cascade.setTargetPosition(newCascadeTarget);
            robot.cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.cascade.setPower(speed);

            timer.reset();

            while (opModeIsActive() && robot.cascade.isBusy())
            {
                telemetry.addData("Completing Action. Time taken:", timer.seconds());
                telemetry.update();
            }

            robot.cascade.setPower(0);
        }
    }

    private void encoderCascadeRetract(double speed, double proportionRetracted)
    {
        if (opModeIsActive())
        {
            int newCascadeTarget = robot.cascade.getCurrentPosition() - (int)(proportionRetracted*6.25*TICKS_PER_ROTATION);

            robot.cascade.setTargetPosition(newCascadeTarget);
            robot.cascade.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.cascade.setPower(-speed);

            timer.reset();

            while (opModeIsActive() && robot.cascade.isBusy())
            {
                telemetry.addData("Completing Action. Time taken:", timer.seconds());
                telemetry.update();
            }

            robot.cascade.setPower(0);
        }
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
}
