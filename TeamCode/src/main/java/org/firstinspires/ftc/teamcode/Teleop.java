package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "TeleOp")
public class Teleop extends LinearOpMode {
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

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRearLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double G1rightStickY = gamepad1.right_stick_y;
            double G1leftStickY = gamepad1.left_stick_y;
            float G1rightTrigger = gamepad1.right_trigger;
            float G1leftTrigger = gamepad1.left_trigger;
            if (gamepad1.dpad_up) {
                speed_control = 1;
            }
            if (gamepad1.dpad_down) {
                speed_control = 0.25f;
            }
            if (gamepad1.dpad_left) {
                speed_control = 0.5f;
            }
            if (gamepad1.dpad_right) {
                speed_control = 0.5f;
            }

            if (G1rightTrigger > 0 && G1leftTrigger == 0) {

                motorFrontLeft.setPower(-G1rightTrigger * speed_control);
                motorRearLeft.setPower(G1rightTrigger * speed_control);
                motorFrontRight.setPower(G1rightTrigger * speed_control);
                motorRearRight.setPower(-G1rightTrigger * speed_control);
            } else if (G1leftTrigger > 0 && G1rightTrigger == 0) {
                motorFrontLeft.setPower(G1leftTrigger * speed_control);
                motorRearLeft.setPower(-G1leftTrigger * speed_control);
                motorFrontRight.setPower(-G1leftTrigger * speed_control);
                motorRearRight.setPower(G1leftTrigger * speed_control);
            } else {

                // how to cube  x = Math.pow(y, 3);
                motorFrontLeft.setPower(G1leftStickY * speed_control);
                motorRearLeft.setPower(G1leftStickY * speed_control);
                motorFrontRight.setPower(G1rightStickY * speed_control);
                motorRearRight.setPower(G1rightStickY * speed_control);
            }
            if (gamepad1.left_bumper) {
                Claw.setPosition(0.0);
            }

            if (gamepad1.right_bumper) {
                Claw.setPosition(1.0);
                idle();
            }
        }
    }
}
