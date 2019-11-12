package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "TeleOp", group = "12806")
public class Teleop extends LinearOpMode {

    Robokenbot robot   = new Robokenbot();

    @Override
    public void runOpMode() throws InterruptedException {
        float speed_control = 1;
        double ArmSpeedControl = 0.4;


        robot.init(hardwareMap);

        robot.motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
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
            if (gamepad2.left_bumper) {
                robot.claw.setPosition(0.0);
                telemetry.addData("Status", "Claw");    //
                telemetry.update();
            }

            if (gamepad2.right_bumper) {
                robot.claw.setPosition(1.0);
                telemetry.addData("Status", "Claw");    //
                telemetry.update();
                idle();

            }

            if (gamepad2.left_stick_y > 0 || gamepad2.left_stick_y < 0) {
                robot.arm.setPower(gamepad2.left_stick_y * ArmSpeedControl);
            }

            if (gamepad2.left_stick_y == 0) {
                robot.arm.setPower(0.0);



            }

            if (gamepad2.y)
            {
                ArmSpeedControl = 0.6;
            }


            if (gamepad2.a)
            {
                ArmSpeedControl = 0.4;
            }


            if (gamepad2.dpad_up){
                robot.intakeServo1.setDirection(CRServo.Direction.FORWARD);
                robot.intakeServo2.setDirection(CRServo.Direction.REVERSE);
                robot.intakeServo1.setPower(0.2);
                robot.intakeServo2.setPower(0.2);
                sleep(500);
                robot.intakeServo1.setPower(0.0);
                robot.intakeServo2.setPower(0.0);

            }
            if (gamepad2.dpad_down){
                robot.intakeServo1.setDirection(CRServo.Direction.REVERSE);
                robot.intakeServo2.setDirection(CRServo.Direction.FORWARD);
                robot.intakeServo1.setPower(0.2);
                robot.intakeServo2.setPower(0.2);
                sleep(500);
                robot.intakeServo1.setPower(0.0);
                robot.intakeServo2.setPower(0.0);


            }


        }
    }
}
