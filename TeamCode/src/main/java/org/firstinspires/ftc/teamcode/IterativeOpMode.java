package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp (name="Basic: IterativeOpMode", group="Iterative OpMode")
public class IterativeOpMode extends OpMode{
    private ElapsedTime timer = new ElapsedTime();
    private HardwarePushbot robot = new HardwarePushbot();

    @Override public void init(){
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized" );
    }

    @Override public void start(){
        timer.reset();
    }

    @Override public void loop(){
        double strafe_Y = -gamepad1.right_stick_y; // forward is pos, backward is neg. (-1 <= magnitude <= 1)
        double strafe_X = gamepad1.right_stick_x; //right is pos, left is neg. (-1 <= magnitude <= 1)
        double intakePwr = 0;
            if(gamepad1.right_bumper)
            { intakePwr = 1; }
        double flipperPwr = 0;
            if (gamepad1.y)
            { flipperPwr = 0.5; }
            else if (gamepad1.a)
            { flipperPwr = -0.5; }
        double carouselPwr = 0;
            if(gamepad1.right_trigger>0)
            { carouselPwr = gamepad1.right_trigger; }
            else if (gamepad1.left_trigger>0)
            { carouselPwr = -gamepad1.left_trigger; }
        double cascadePwr = -gamepad2.left_stick_y; // forward is pos, backward is neg. (-1 <= magnitude <= 1)
        double rotatorPwr = gamepad2.right_stick_x; //right is pos, left is neg. (-1 <= magnitude <= 1)
        /*
        Does the CRServo rotator accept negative inputs for power,
        or do we need to change the servo's direction to get it to turn the opposite way?
         */
        double capPosition = robot.cap.getPosition();
            if (gamepad2.x)
            {capPosition = HardwarePushbot.CAP_CLOSED;}
            else if (gamepad2.a)
            {capPosition = HardwarePushbot.CAP_OPEN;}
        double armPosition = robot.arm.getPosition();
            if (gamepad2.dpad_up)
            { armPosition=HardwarePushbot.ARM_OUT;}
            else if (gamepad2.dpad_down)
            { armPosition=HardwarePushbot.ARM_IN;}
        /*
        Do the servos move completely to the previously assigned position before listening to new commands?
        */

        telemetry.addData("X", strafe_X );
        telemetry.addData("Y", strafe_Y );

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

        robot.intake.setPower(intakePwr);
        robot.flipper.setPower(flipperPwr);
        robot.carousel.setPower(carouselPwr);
        robot.rotator.setPower(rotatorPwr);
        robot.cascade.setPower(cascadePwr);

        robot.arm.setPosition(armPosition);
        robot.cap.setPosition(capPosition);
    }
}
