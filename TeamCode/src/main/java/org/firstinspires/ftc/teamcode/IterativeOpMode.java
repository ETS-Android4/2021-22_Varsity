package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp (name="Basic: IterativeOpMode", group="Iterative OpMode")
public class IterativeOpMode extends OpMode{
    private ElapsedTime flipperTimer = null;
    private HardwarePushbot robot = new HardwarePushbot();

    @Override public void init(){
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized" );
    }

    @Override public void start(){ }

    @Override public void loop(){
        double strafe_Y = gamepad1.left_stick_y; // forward is pos, backward is neg. (-1 <= magnitude <= 1)
        double strafe_X = gamepad1.right_stick_x; //right is pos, left is neg. (-1 <= magnitude <= 1)
       robot.intake.setPower(0);
            if(gamepad1.right_bumper)
            { robot.intake.setPower(0.5);}

            if (gamepad1.y) {
                flipperTimer = new ElapsedTime();
                robot.flipper.setPower(-0.5);
            }
            else if (gamepad1.a) {
                flipperTimer = new ElapsedTime();
                robot.flipper.setPower(0.3);
            }
            else if (flipperTimer!=null && robot.flipper.getPower()==-0.5 && flipperTimer.milliseconds()>=300)
            {//going up
                robot.flipper.setPower(-0.2);
            }
            else if (flipperTimer!=null && robot.flipper.getPower()==-0.2 && flipperTimer.milliseconds()>=500)
            {//end of going up
                robot.flipper.setPower(-0.1);
                robot.intake.setPower(0.2);
                flipperTimer.reset();
            }
            else if (flipperTimer!=null && robot.intake.getPower()>0 && flipperTimer.milliseconds()>=400)
            {//intake spinning after going up
                robot.intake.setPower(0);
                robot.flipper.setPower(0);
                flipperTimer=null;
            }
            else if (flipperTimer!=null &&  robot.flipper.getPower()>0 &&flipperTimer.milliseconds()>=300)
            {//going down
                robot.flipper.setPower(0);
                flipperTimer=null;
            }
        double carouselPwr = 0;
            if(gamepad1.right_trigger>0)
            { carouselPwr = gamepad1.right_trigger; }
            else if (gamepad1.left_trigger>0)
            { carouselPwr = -gamepad1.left_trigger; }
        double cascadePwr = -gamepad2.right_stick_y; // forward is pos, backward is neg. (-1 <= magnitude <= 1)
        double rotatorPwr=0;
        if (gamepad2.right_bumper && !gamepad2.left_bumper)
        {
            rotatorPwr = -1;
        }
        else if(gamepad2.left_bumper && !gamepad2.right_bumper)//right is pos, left is neg. (-1 <= magnitude <= 1)
        {
            rotatorPwr=-1;
        }
            if (gamepad2.b)
            { robot.cap.setPosition(HardwarePushbot.CAP_OPEN); }
            else if( gamepad2.x)
            { robot.cap.setPosition(HardwarePushbot.CAP_CLOSED);}
            if (gamepad2.dpad_up)
            { robot.arm.setPosition(HardwarePushbot.ARM_OUT); }
            else if( gamepad2.dpad_down)
            { robot.arm.setPosition(HardwarePushbot.ARM_IN);}
        /*
        Does the CRServo rotator accept negative inputs for power,
        or do we need to change the servo's direction to get it to turn the opposite way?
         */

        double blStrafePwr = (strafe_X+strafe_Y);
        double brStrafePwr = -(strafe_X-strafe_Y);
        double flStrafePwr = -(strafe_X-strafe_Y);
        double frStrafePwr = (strafe_X+strafe_Y);

        double rotate = gamepad1.left_stick_x;

        //CW is pos (joystick right?), CCW is neg (joystick left?)
        telemetry.addData("Rotate", rotate );

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
        robot.cascade.setPower(cascadePwr);

    }
}
