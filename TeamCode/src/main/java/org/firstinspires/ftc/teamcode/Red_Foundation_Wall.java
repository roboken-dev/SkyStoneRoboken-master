package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous (name = "RedFoundationWall", group = "12806")

public class Red_Foundation_Wall extends LinearOpMode {

    Robokenbot robot   = new Robokenbot();

    public void runOpMode() throws InterruptedException {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap,this);

        waitForStart();
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        robot.claw.setPosition((1.0));

        robot.encoderDrive(robot.DRIVE_SPEED,  -15,-15,50.0, this);  // S1: Forward 24 Inches with 5 Sec timeout

//        robot.initRunWithoutEncoder();
        robot.strafeLeftByTime(0.4, 2100);

        robot.encoderDrive(robot.DRIVE_SPEED, -15,-15,30.0, this);
        robot.claw.setPosition((0.0));
        sleep(1000);

        robot.encoderDrive(robot.DRIVE_SPEED /5, 29.5, 29.5, 30.0, this);
        robot.claw.setPosition((1.0));//claw goes up

        robot.strafeRightByTime(0.5, 3600); //straf5es out of the way

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }



}
