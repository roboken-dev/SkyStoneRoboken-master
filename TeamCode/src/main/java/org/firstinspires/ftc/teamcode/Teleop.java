package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "TeleOp")
public class Teleop extends LinearOpMode {

    Robokenbot robot   = new Robokenbot();

    @Override
    public void runOpMode() throws InterruptedException {
        float speed_control = 1;

        robot.init(hardwareMap);

        robot.motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.motorRearLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Ready to Go");    //
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double G1rightStickY = gamepad1.right_stick_y;
            double G1leftStickY = gamepad1.left_stick_y;
            float G1rightTrigger = gamepad1.right_trigger;
            float G1leftTrigger = gamepad1.left_trigger;
            if (gamepad1.dpad_up) {
                speed_control = 1;
                telemetry.addData("Status", "Setting Speed to 1");    //
                telemetry.update();
            }
            if (gamepad1.dpad_down) {
                speed_control = 0.25f;
                telemetry.addData("Status", "Setting Speed to .25");    //
                telemetry.update();
            }
            if (gamepad1.dpad_left) {
                speed_control = 0.5f;
                telemetry.addData("Status", "Setting Speed to .5");    //
                telemetry.update();
            }
            if (gamepad1.dpad_right) {
                speed_control = 0.5f;
                telemetry.addData("Status", "Setting Speed to .5");    //
                telemetry.update();
            }

            if (G1rightTrigger > 0 && G1leftTrigger == 0) {

                robot.motorFrontLeft.setPower(-G1rightTrigger * speed_control);
                robot.motorRearLeft.setPower(G1rightTrigger * speed_control);
                robot.motorFrontRight.setPower(G1rightTrigger * speed_control);
                robot.motorRearRight.setPower(-G1rightTrigger * speed_control);

                telemetry.addData("Status", "Strafing Right");    //
                telemetry.update();

            } else if (G1leftTrigger > 0 && G1rightTrigger == 0) {
                robot.motorFrontLeft.setPower(G1leftTrigger * speed_control);
                robot.motorRearLeft.setPower(-G1leftTrigger * speed_control);
                robot.motorFrontRight.setPower(-G1leftTrigger * speed_control);
                robot.motorRearRight.setPower(G1leftTrigger * speed_control);

                telemetry.addData("Status", "Strafing Left");    //
                telemetry.update();

            } else {

                // how to cube  x = Math.pow(y, 3);
                robot.motorFrontLeft.setPower(G1leftStickY * speed_control);
                robot.motorRearLeft.setPower(G1leftStickY * speed_control);
                robot.motorFrontRight.setPower(G1rightStickY * speed_control);
                robot.motorRearRight.setPower(G1rightStickY * speed_control);

                telemetry.addData("Status", "Moving");    //
                telemetry.update();

            }
            if (gamepad1.left_bumper) {
                robot.Claw.setPosition(0.0);
                telemetry.addData("Status", "Claw");    //
                telemetry.update();

            }

            if (gamepad1.right_bumper) {
                robot.Claw.setPosition(1.0);
                telemetry.addData("Status", "Claw");    //
                telemetry.update();
                idle();
            }
        }
    }
}
