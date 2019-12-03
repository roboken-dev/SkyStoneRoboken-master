package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;


@Autonomous (name = "RedSeekSkystone", group = "12806")

public class RedSkystoneV1 extends LinearOpMode {

    Robokenbot robot   = new Robokenbot();

    public void runOpMode() throws InterruptedException {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap,this);
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        waitForStart();

        robot.claw.setPosition((1.0));
        robot.encoderDrive(0.4,-28,-28,8000,this);

        // step 1 - encoder drive toward the wall of stones
//        robot.encoderDrive(robot.DRIVE_SPEED,  -5,-5,50.0, this);

        // step 2 - strafe right toward the wall to park in front of the first stone (we may shave time also if we want to park in front of the 2nd stone from the wall)

        // step 3 - seek the Skystone by strafing until the color sensor sees black
        robot.strafeRightByTime(0.4,1000);

        robot.strafeLeft(0.25);

        do {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
                    (int) (robot.sensorColor.green() * SCALE_FACTOR),
                    (int) (robot.sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", robot.sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();

        } while (hsvValues[0] < 101);

        robot.stopDriving();

        robot.strafeLeftByTime(0.1,1000); // try to center robot in front of Skystone

        // step 4 - grab the Skystone. We may need to move forward a tad to position the robot.
        robot.encoderDrive(0.1,-3,-3,4000,this);
        robot.claw.setPosition((0.0));
        sleep(1000);


        // step 5 - back the robot away a tad from wall of stones, to avoid hitting the Skybridge pylon in the next step
        // May also want to turn 90 degrees, depending on claw technique. If we turn, then we encoder drive using distance for remaining steps.  If we can avoid turning, we strafe instead.
        robot.encoderDrive(0.2,6,6,5000,this);
        robot.rotate(-90, .2, true, this);
        robot.driveForward(-0.2);
        do {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (robot.bottomSensorColor.red() * SCALE_FACTOR),
                    (int) (robot.bottomSensorColor.green() * SCALE_FACTOR),
                    (int) (robot.bottomSensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", robot.sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();

        } while (hsvValues[0] < 101);
        robot.stopDriving();
        robot.encoderDrive(0.5,-18,-18,5000,this);
        robot.claw.setPosition((1.0));
        robot.strafeRightByTime(0.2,500); // try to center robot in front of Skystone
        robot.encoderDrive(0.4,20,20,8000,this);


        // step 6 - Seek SkyBridge - strafe toward the Skybridge using downward color sensor

        // step 7 - Continue past Skybridge and into Building area

        // step 8 - Release the SkyStone

        // step 9 - Seek SkyBridge using downward color sensor and park

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
