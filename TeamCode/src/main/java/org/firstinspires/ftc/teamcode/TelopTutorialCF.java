/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TelopTutorialCF", group = "Tutorials")
public class TelopTutorialCF extends LinearOpMode {

    private DcMotor motorLeft;
    private DcMotor motorRight;
    private Servo Claw;

    @Override
    public void runOpMode() throws InterruptedException {
        motorLeft = hardwareMap.dcMotor.get("left");
        motorRight = hardwareMap.dcMotor.get("right");
        Claw = hardwareMap.servo.get("servo");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {


            motorLeft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);

            if (gamepad1.left_bumper) {
                Claw.setPosition(0.0);
            }

            if (gamepad1.right_bumper) {
                Claw.setPosition(1.0);
            }

            idle();
        }
    }

*/
