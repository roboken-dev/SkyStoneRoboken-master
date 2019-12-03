package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "auto9-21", group = "12806")
public class Auto9_21 extends LinearOpMode {

    Robokenbot robot   = new Robokenbot();


    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap,this);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Init");    //
        telemetry.update();

        float speed_control = 1;


        robot.motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.motorRearLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {


            robot.motorFrontLeft.setPower(0.25 * speed_control);
            robot.motorRearLeft.setPower(0.25 * speed_control);
            robot.motorFrontRight.setPower(0.25 * speed_control);
            robot.motorRearRight.setPower(0.25 * speed_control);

            Thread.sleep(3800);
            robot.motorFrontLeft.setPower(0);
            robot.motorRearLeft.setPower(0);
            robot.motorFrontRight.setPower(0);
            robot.motorRearRight.setPower(0);
            Thread.sleep(300000);
        }
    }
}
