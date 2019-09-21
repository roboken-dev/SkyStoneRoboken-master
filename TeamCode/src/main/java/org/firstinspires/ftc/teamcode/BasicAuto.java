package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name = "BasicAuto")

public class BasicAuto extends LinearOpMode {

    DcMotor motorFrontLeft;  // motor1
    DcMotor motorRearLeft;  // motor 2
    DcMotor motorFrontRight; // motor 3
    DcMotor motorRearRight; // motor 4

    DigitalChannel digitalTouch;  // Hardware Device Object
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    double DRIVE_POWER = 0.2;

    public void runOpMode() throws InterruptedException {

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorRearLeft = hardwareMap.dcMotor.get("motorRearLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorRearRight = hardwareMap.dcMotor.get("motorRearRight");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        waitForStart();

        DriveTillDistance(DRIVE_POWER, 12);

        DriveForwardTime(-.5, 1000);

        TurnLeftTime(DRIVE_POWER, 1000);

        DriveForwardTime(DRIVE_POWER, 500);

        TurnRightTime(DRIVE_POWER, 1500);

        DriveForwardTime(DRIVE_POWER, 8000);

        StopDriving();
    }

    public void DriveForward(double power) {
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
    }

    public void StopDriving() {
        DriveForward(0);
    }

    public void TurnLeft(double power) {
        motorFrontLeft.setPower(-power);
        motorFrontRight.setPower(power);
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


    public void DriveTillTouch(double power) {
        while (digitalTouch.getState() == true) {
            DriveForward(power);
        }

    }

    public void DriveTillDistance(double power, double distance) {
        double howfar;

        while (Double.isNaN(sensorDistance.getDistance(DistanceUnit.CM)) || sensorDistance.getDistance(DistanceUnit.CM) > distance) {
            DriveForward(power);
        }


    }
/*
    public void StrafeLeftTIme(double power, long time) throws InterruptedException {
        StrafeLeft(power);
        Thread.sleep(time);
    }

    public void StrafeRightTime(double power, long time) throws InterruptedException {
        StrafeRight(power);
        Thread.sleep(time);
    }

    public void StrafeRight(double power) {
        motorFrontLeft.setPower(-power);
        motorRearLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorRearRight.setPower(-power);
    }

    public void StrafeRight(double power) {
        motorFrontLeft.setPower(power);
        motorRearLeft.setPower(-power);
        motorFrontRight.setPower(-power);
        motorRearRight.setPower(power);
    }

 */
}