package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous (name = "BlueFoundationWall", group = "12806")

public class Blue_Foundation_Wall extends LinearOpMode {

    Robokenbot robot   = new Robokenbot();

    static final double     DRIVE_SPEED             = 0.3;

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

        robot.encoderDrive(DRIVE_SPEED,  -12,-12,50.0, this);  // S1: Forward 24 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //robot.Claw.setPosition(1.0);
        //encoderDrive(DRIVE_SPEED, 32, 32, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

//        robot.initRunWithoutEncoder();
        robot.strafeRightByTime(0.4, 1900);

        robot.encoderDrive(DRIVE_SPEED, -15,-15,30.0, this);
        robot.claw.setPosition((0.0));
        sleep(1000);

        robot.encoderDrive(DRIVE_SPEED /4, 29.5, 29.5, 30.0, this);
        robot.claw.setPosition((1.0));//claw goes up

        robot.strafeLeftByTime(0.5, 3800); //straf5es out of the way

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
