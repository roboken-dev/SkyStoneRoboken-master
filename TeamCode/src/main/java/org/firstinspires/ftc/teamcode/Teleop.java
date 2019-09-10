package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "TeleOp")
public class Teleop extends LinearOpMode
{
 private DcMotor motorFrontLeft;
 private DcMotor motorFrontRight;
 private DcMotor motorRearLeft;
 private DcMotor motorRearRight;



    @Override
 public void runOpMode() throws InterruptedException

    {
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorRearLeft = hardwareMap.dcMotor.get("motorRearLeft");
        motorRearRight = hardwareMap.dcMotor.get("motorRearRight");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRearLeft.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        while(opModeIsActive())
        {
            motorFrontLeft.setPower(-gamepad1.left_stick_y);
            motorFrontRight.setPower(-gamepad1.right_stick_y);
            motorRearLeft.setPower(-gamepad1.left_stick_y);
            motorRearRight.setPower(-gamepad1.right_stick_y);

            idle();

        }
    }
}
