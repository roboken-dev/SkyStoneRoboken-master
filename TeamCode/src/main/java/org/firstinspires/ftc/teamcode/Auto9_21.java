package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "auto9-21")
public class Auto9_21 extends LinearOpMode {
    private DcMotor motorFrontLeft;  // motor1
    private DcMotor motorRearLeft;  // motor 2
    private DcMotor motorFrontRight; // motor 3
    private DcMotor motorRearRight; // motor 4
    private Servo Claw;

    @Override
    public void runOpMode() throws InterruptedException {
        float speed_control = 1;

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorRearLeft = hardwareMap.dcMotor.get("motorRearLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorRearRight = hardwareMap.dcMotor.get("motorRearRight");
        Claw = hardwareMap.servo.get("servo");

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorRearLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {


            motorFrontLeft.setPower(0.25 * speed_control);
            motorRearLeft.setPower(0.25 * speed_control);
            motorFrontRight.setPower(0.25 * speed_control);
            motorRearRight.setPower(0.25 * speed_control);

            Thread.sleep(3800);
            motorFrontLeft.setPower(0);
            motorRearLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorRearRight.setPower(0);
            Thread.sleep(300000);
        }
    }
}
