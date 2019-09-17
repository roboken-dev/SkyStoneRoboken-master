package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "TeleOp")
public class Teleop extends LinearOpMode
{
 private DcMotor motorFrontLeft;  // motor1
 private DcMotor motorRearLeft;  // motor 2
 private DcMotor motorFrontRight; // motor 3
 private DcMotor motorRearRight; // motor 4



    @Override
 public void runOpMode() throws InterruptedException

    {
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorRearLeft = hardwareMap.dcMotor.get("motorRearLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorRearRight = hardwareMap.dcMotor.get("motorRearRight");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRearLeft.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        while(opModeIsActive())
        {
            motorFrontLeft.setPower(-gamepad1.left_stick_y);
            motorRearLeft.setPower(-gamepad1.left_stick_y);
            motorFrontRight.setPower(-gamepad1.right_stick_y);
            motorRearRight.setPower(-gamepad1.right_stick_y);

            idle();
/// test 11
        }
    }
}
