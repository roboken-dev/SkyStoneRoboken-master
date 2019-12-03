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

        // step 1 - encoder drive toward the wall of stones
//        robot.encoderDrive(robot.DRIVE_SPEED,  -5,-5,50.0, this);

        // step 2 - strafe right toward the wall to park in front of the first stone (we may shave time also if we want to park in front of the 2nd stone from the wall)

        // step 3 - seek the Skystone by strafing until the color sensor sees black
        robot.strafeLeft(0.2);

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
        robot.claw.setPosition((0.0));
        sleep(1000);

        // step 5 - back the robot away a tad from wall of stones, to avoid hitting the Skybridge pylon in the next step
        // May also want to turn 90 degrees, depending on claw technique. If we turn, then we encoder drive using distance for remaining steps.  If we can avoid turning, we strafe instead.

        robot.rotate(90, .2, this);

        // step 6 - Seek SkyBridge - strafe toward the Skybridge using downward color sensor

        // step 7 - Continue past Skybridge and into Building area

        // step 8 - Release the SkyStone

        // step 9 - Seek SkyBridge using downward color sensor and park

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

}
