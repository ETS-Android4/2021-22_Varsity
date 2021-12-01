package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name="Basic: IterativeOpMode", group="Iterative OpMode")
public class IterativeOpMode extends OpMode{
    private ElapsedTime timer = new ElapsedTime();
    private HardwarePushbot robot = new HardwarePushbot();

    private static final double maxSpeed = 1;

    @Override public void init(){
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized" );
    }

    @Override public void start(){
        timer.reset();
    }

    @Override public void loop(){
        double strafe_Y = -gamepad1.right_stick_y; // forward is pos, backward is neg. (-1 <= magnitude <= 1)
        //strafe_Y *= maxSpeed; //I don't think a max speed is even necessary, because the math gets so screwy...
        double strafe_X = gamepad1.right_stick_x; //right is pos, left is neg. (-1 <= magnitude <= 1)
        //strafe_X *= maxSpeed;
        //note to self: make sure sticks return neg, pos & magnitude correctly
        telemetry.addData("X", strafe_X );
        telemetry.addData("Y", strafe_Y );

        double blStrafePwr = (strafe_X+strafe_Y)/2;
        double brStrafePwr = -(strafe_X-strafe_Y)/2;
        double flStrafePwr = -(strafe_X-strafe_Y)/2;
        double frStrafePwr = (strafe_X+strafe_Y)/2;

        double rotate;
        if(gamepad1.dpad_left)
        {
            rotate=-1;
        }
        else if (gamepad1.dpad_right){
            rotate=1;
        }
        else{
            rotate=0;
        }
        //CW is pos (joystick right?), CCW is neg (joystick left?)
        telemetry.addData("Rotate", rotate );

        if (rotate!=0) {
            robot.blDrive.setPower(blStrafePwr * rotate);
            robot.brDrive.setPower(brStrafePwr * -rotate);
            robot.flDrive.setPower(flStrafePwr * rotate);
            robot.frDrive.setPower(frStrafePwr * -rotate);
        }
        else{
            robot.blDrive.setPower(blStrafePwr);
            robot.brDrive.setPower(brStrafePwr);
            robot.flDrive.setPower(flStrafePwr);
            robot.frDrive.setPower(frStrafePwr);
        }
        //issues with left joystick. Robot doesn't rotate, only strafes.
    }
}
