package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous (name = "Blue_Park", group = "12806")

public class Blue_Park extends LinearOpMode {

    Robokenbot robot   = new Robokenbot();

    public void runOpMode() throws InterruptedException {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap,this);

        waitForStart();

        robot.claw.setPosition((1.0));

        robot.encoderDrive(robot.DRIVE_SPEED,  -25,-25,50.0, this);  // S1: Forward 24 Inches with 5 Sec timeout

        robot.strafeRightByTime(0.4, 2500);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
