/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.SynchronousOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name="TutorialAutonomousCF")
// @Disabled
public class TutorialAutonomousCF extends LinearOpMode {

    // Declare Motors
    DcMotor motorLeft = null;
    DcMotor motorRight = null;

    DigitalChannel digitalTouch;  // Hardware Device Object
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize motors
        motorLeft = hardwareMap.dcMotor.get("left");
        motorRight = hardwareMap.dcMotor.get("right");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
       */
/*
       DriveForwardTime(DRIVE_POWER,4000);
       TurnLeftTime(DRIVE_POWER,500);
       DriveForwardTime(DRIVE_POWER,4000);
       TurnRightTime(DRIVE_POWER,500);
       DriveForwardTime(DRIVE_POWER,4000);
       *//*

        // DriveTillTOuch(1);
        while (opModeIsActive()) {
            // send the info back to driver station using telemetry function.

            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.update();
        }
        DriveTillDistance(DRIVE_POWER, 12);
        DriveForwardTime(-.5, 1000);
        TurnLeftTime(DRIVE_POWER, 1000);
        DriveForwardTime(DRIVE_POWER, 500);
        TurnRightTime(DRIVE_POWER, 1500);
        DriveForwardTime(DRIVE_POWER, 8000);

        StopDriving();
// Complete Objective
    }

    double DRIVE_POWER = 0.2;

    public void DriveForward(double power) {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    public void StopDriving() {
        DriveForward(0);
    }

    public void TurnLeft(double power) {
        motorLeft.setPower(-power);
        motorRight.setPower(power);
    }

    public void TurnRight(double power) {
        TurnLeft(-power);
    }

    public void DriveForwardTime(double power, long time) throws InterruptedException {
        DriveForward(power);
        Thread.sleep(time);
    }

    public void TurnLeftTime(double power, long time) throws InterruptedException {
        TurnLeft(power);
        Thread.sleep(time);
    }

    public void TurnRightTime(double power, long time) throws InterruptedException {
        TurnRight(power);
        Thread.sleep(time);
    }

    public void DriveTillTOuch(double power) {
        while (digitalTouch.getState() == true) {
            DriveForward(power);
        }

    }

    public void DriveTillDistance(double power, double distance) {
        double howfar;

        while (Double.isNaN(sensorDistance.getDistance(DistanceUnit.CM)) || sensorDistance.getDistance(DistanceUnit.CM) > distance) {
            DriveForward(power);
        }

        public void StrafeLeftTIme( double power, long time) throws InterruptedException
        {
            StrafeLeft(power);
            Thread.sleep(time);
        }

        public void StrafeRightTime( double power, long time) throws InterruptedException
        {
            StrafeRight(power);
            Thread.sleep(time);
        }
        public void StrafeRight( double power)
        {
            FrontLeft.setPower(-power);
            RearLeft.setPower(power);
            FrontRight.setPower(power);
            RearRight.setPower(-power);


        }


    }

}













*/
