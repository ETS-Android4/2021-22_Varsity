package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name="Basic: IterativeOpMode", group="Iterative OpMode")
public class IterativeOpMode extends OpMode{
    private ElapsedTime timer = new ElapsedTime();
    private int flipperHighPosition = 80;
    private int flipperMidPosition = 175;
    private int flipperLowPosition = 300;
    private HardwarePushbot robot = new HardwarePushbot();
    private boolean intakeGoingDown = false;
    private boolean intakeGoingUp = false;

    @Override public void init(){
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized" );
        robot.flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override public void start(){ }

    @Override public void loop(){

        double strafe_Y = gamepad1.left_stick_y; // forward is pos, backward is neg. (-1 <= magnitude <= 1)
        double strafe_X = -gamepad1.right_stick_x; //right is pos, left is neg. (-1 <= magnitude <= 1)

        double carouselPwr = 0;
            if(gamepad1.right_trigger>0)
            { carouselPwr = gamepad1.right_trigger; }
            else if (gamepad1.left_trigger>0)
            { carouselPwr = -gamepad1.left_trigger; }
        double rotatorPwr=0;
        if (gamepad2.right_bumper && !gamepad2.left_bumper)
        {
            rotatorPwr = -0.5;
        }
        else if(gamepad2.left_bumper && !gamepad2.right_bumper)//right is pos, left is neg. (-1 <= magnitude <= 1)
        {
            rotatorPwr=0.5;
        }
        double cascadePwr = -gamepad2.right_stick_y; // forward is pos, backward is neg. (-1 <= magnitude <= 1)
            if( gamepad2.x)
            {
                robot.cap.setPosition(HardwarePushbot.CAP_CLOSED);
                robot.arm.setPosition(HardwarePushbot.ARM_OUT);
                robot.cap.setPosition(HardwarePushbot.CAP_OPEN);
                robot.arm.setPosition(HardwarePushbot.ARM_IN);
            }

        double blStrafePwr = (strafe_X+strafe_Y);
        double brStrafePwr = -(strafe_X-strafe_Y);
        double flStrafePwr = -(strafe_X-strafe_Y);
        double frStrafePwr = (strafe_X+strafe_Y);

        double rotate = gamepad1.left_stick_x;

        //CW is pos (joystick right?), CCW is neg (joystick left?)

        if (strafe_Y==0 && strafe_X==0) {
            robot.blDrive.setPower(rotate);
            robot.brDrive.setPower(-rotate);
            robot.flDrive.setPower(rotate);
            robot.frDrive.setPower(-rotate);
        }
        else {
            robot.blDrive.setPower(blStrafePwr);
            robot.brDrive.setPower(brStrafePwr);
            robot.flDrive.setPower(flStrafePwr);
            robot.frDrive.setPower(frStrafePwr);
        }

        robot.carousel.setPower(carouselPwr);
        robot.rotator.setPower(rotatorPwr);
        //robot.cascade.setPower(cascadePwr);

        if (gamepad2.left_trigger>0)
        {
            robot.flipper.setVelocity((int)(gamepad2.left_trigger*1000));
        }
        else if (gamepad2.right_trigger>0)
        {
            robot.flipper.setVelocity(-1000*gamepad2.right_trigger);
        }

        if(gamepad1.x)
        { intakeGoingDown=true;
        intakeGoingUp=false;}
        if (gamepad1.b)
        { intakeGoingUp = true;
        intakeGoingDown=false;}

        if(intakeGoingDown)
        {
            if(robot.flipper.getCurrentPosition()<flipperHighPosition) {
                robot.flipper.setPower(0.1);
            }
            else if (robot.flipper.getCurrentPosition()>flipperLowPosition) {
                robot.flipper.setPower(0);
                intakeGoingDown=false;
            }
            else if (robot.flipper.getCurrentPosition()>flipperHighPosition) {
                robot.flipper.setPower(0.01);
            }
        }
        if(intakeGoingUp)
        {
            if(robot.flipper.getCurrentPosition()>flipperLowPosition)
            {
                robot.flipper.setPower(-0.3);
            }
            else if (robot.flipper.getCurrentPosition()<flipperHighPosition-10)
            {
                robot.flipper.setPower(0.005);
            }
            else if (robot.flipper.getCurrentPosition()<160)
            {
                robot.flipper.setPower(0);
            }
        }

        telemetry.addData("Status:" ,"current flipper position=%d, current flipper velocity=%f",
                robot.flipper.getCurrentPosition(), robot.flipper.getPower());
        telemetry.addData("Status:","FlipperGoingUp? %b Down? %b", intakeGoingUp, intakeGoingDown);
        telemetry.update();

        /*if (gamepad1.left_bumper) {
            flipperMinPosition += 10;
        } else if (gamepad1.right_bumper) {
            flipperMaxPosition += 10;
        }

        if (gamepad1.x) {
            flipperMinPosition -= 10;
        } else if (gamepad1.b) {
            flipperMaxPosition -= 10;
        }
        telemetry.addData("Flipper Status:", "CurrentPosition=%d, TargetPosition=%d",
                robot.flipper.getCurrentPosition(), robot.flipper.getTargetPosition());
        telemetry.update();*/
    }

    private void cascadeProcedure()
    {

        robot.arm.setPosition(HardwarePushbot.ARM_OUT);

        timer.reset(); //cascade out
        robot.cascade.setPower(0.5);
        while (timer.milliseconds()<1200)
        { telemetry.addData("Status:", "extending cascade");
            telemetry.update();}
        robot.cascade.setPower(0);

        robot.cap.setPosition(0.8);//CAP_CLOSED and CAP_OPEN reversed, I think. Must fix in HardwarePushbot.

        robot.cascade.setPower(-0.7); //cascade in
        timer.reset();
        while (timer.milliseconds()<1000)
        { telemetry.addData("Status:", "retracting cascade");
            telemetry.update();}
        robot.cascade.setPower(0);

        robot.arm.setPosition(HardwarePushbot.ARM_IN);
    }
}
