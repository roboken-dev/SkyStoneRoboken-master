package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous (name = "RedFoundationOff", group = "12806")

public class Red_Foundation_Off extends LinearOpMode {

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

//        robot.initRunWithoutEncoder();
        robot.strafeLeftByTime(0.4, 2100);

        robot.encoderDrive(DRIVE_SPEED, -15,-15,30.0, this);
        robot.claw.setPosition((0.0));
        sleep(1000);

        robot.encoderDrive(DRIVE_SPEED /4, 29.5, 29.5, 30.0, this);
        robot.claw.setPosition((1.0));//claw goes up

        robot.strafeRightByTime(0.5, 2400);
        robot.encoderDrive(DRIVE_SPEED, -24,-24,30.0, this);
        robot.strafeRightByTime(0.5,1300);

        //straf5es out of the way
        /*encoderDrive(DRIVE_SPEED,-20,-20,10.0);

        strafeLeftByTime(0.4,2000);

        encoderDrive(DRIVE_SPEED,-7,-7,10.0);

        strafeRightByTime(0.4,2500);*/


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
