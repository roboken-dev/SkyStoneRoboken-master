package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous (name = "Red_Park", group = "12806")

public class Red_Park extends LinearOpMode {

    Robokenbot robot   = new Robokenbot();

    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.1;
    double DRIVE_POWER = 1;

    public void runOpMode() throws InterruptedException {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        waitForStart();
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        robot.claw.setPosition((1.0));

        robot.encoderDrive(DRIVE_SPEED,  -20,-20,50.0, this);  // S1: Forward 24 Inches with 5 Sec timeout

//        robot.initRunWithoutEncoder();
        robot.strafeLeftByTime(0.4, 2500);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
