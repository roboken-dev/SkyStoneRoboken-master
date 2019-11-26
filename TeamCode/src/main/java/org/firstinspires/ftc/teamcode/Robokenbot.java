package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



public class Robokenbot
{
    public DcMotor motorFrontLeft;  // motor1
    public DcMotor motorRearLeft;  // motor 2
    public DcMotor motorFrontRight; // motor 3
    public DcMotor motorRearRight; // motor 4
    public Servo claw;
    public CRServo arm;
    public DigitalChannel digitalTouch;  // Hardware Device Object
    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;
    public CRServo intakeServo1;
    public CRServo intakeServo2;
    private ElapsedTime     runtime = new ElapsedTime();

    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;



    static final double     SCALE_FACTOR = 75.0/75.0; //  if drive speed = .2 or .3 use 75.0/75.0;  .5 is 75.0/76.0 .4 is 75.0/75.5 if drive_speed = .1, use 1.0; if drive_speed = .3, use 75.0/77.0 note that .3 has hard time braking
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (SCALE_FACTOR * COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;


    HardwareMap hwMap = null;
    private ElapsedTime period =new ElapsedTime();

    public Robokenbot(){ }


    public void init(HardwareMap ahwMap) {

        hwMap=ahwMap;
        motorFrontLeft = hwMap.dcMotor.get("motorFrontLeft");
        motorRearLeft = hwMap.dcMotor.get("motorRearLeft");
        motorFrontRight = hwMap.dcMotor.get("motorFrontRight");
        motorRearRight = hwMap.dcMotor.get("motorRearRight");
        //       digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // get a reference to the color sensor.
        sensorColor = hwMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hwMap.get(DistanceSensor.class, "sensor_color_distance");

        claw = hwMap.servo.get("servo");
        arm=hwMap.crservo.get("arm");
        intakeServo1=hwMap.crservo.get("intakeServo1");
        intakeServo2=hwMap.crservo.get("intakeServo2");
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, LinearOpMode opmode) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;

        // Send telemetry message to signify robot waiting;
        opmode.telemetry.addData("Status", "Resetting Encoders");    //
        opmode.telemetry.update();

        initRunWithEncoder();

        // Send telemetry message to indicate successful Encoder reset
        opmode.telemetry.addData("Path0",  "Starting at %7d :%7d",
                motorFrontLeft.getCurrentPosition(),
                motorFrontRight.getCurrentPosition(),
                motorRearLeft.getCurrentPosition(),
                motorRearRight.getCurrentPosition());
        opmode.telemetry.update();

        // Ensure that the opmode is still active
        if (opmode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = (motorFrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
            newFrontRightTarget = (motorFrontRight.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH));
            newRearLeftTarget = -(motorRearLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
            newRearRightTarget = -(motorRearRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH));

            motorFrontLeft.setTargetPosition(newFrontLeftTarget);
            motorFrontRight.setTargetPosition(newFrontRightTarget);
            motorRearLeft.setTargetPosition(newRearLeftTarget);
            motorRearRight.setTargetPosition(newRearRightTarget);

            // Turn On RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();

            motorFrontLeft.setPower(Math.abs(speed));
            motorFrontRight.setPower(Math.abs(speed));
            motorRearLeft.setPower(Math.abs(speed));
            motorRearRight.setPower(-Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opmode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFrontLeft.isBusy() && motorFrontRight.isBusy() && motorRearLeft.isBusy() && motorRearRight.isBusy())) {

                // Display it for the driver.
                opmode.telemetry.addData("Path1",  "Running to %7d :%7d : %d : %d", newFrontLeftTarget,  newFrontRightTarget, newRearLeftTarget, newRearRightTarget);
                opmode.telemetry.addData("Path2",  "Running at %7d :%7d : %7d : %7d",
                        motorFrontLeft.getCurrentPosition(),
                        motorFrontRight.getCurrentPosition(),
                        motorRearLeft.getCurrentPosition(),
                        motorRearRight.getCurrentPosition());
                opmode.telemetry.update();

            }

            // Stop all motion;
            motorFrontLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorRearLeft.setPower(0);
            motorRearRight.setPower(0);

            initRunWithoutEncoder();
            // Turn off RUN_TO_POSITION

        }
    }

    public void initRunWithEncoder()
    {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorRearRight.setDirection(DcMotor.Direction.FORWARD);
        motorRearLeft.setDirection(DcMotor.Direction.FORWARD);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorRearLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    public void initRunWithoutEncoder()
    {
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorRearRight.setDirection(DcMotor.Direction.REVERSE);
        motorRearLeft.setDirection(DcMotor.Direction.FORWARD);
    }



    public void driveForward(double power) {
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
    }

    public void stopDriving() {
        driveForward(0);
    }

    public void turnLeft(double power) {
        motorFrontLeft.setPower(-power);
        motorFrontRight.setPower(power);
    }

    public void turnRight(double power) {
        turnLeft(-power);
    }

    public void driveForwardByTime(double power, long time) throws InterruptedException {
        driveForward(power);
        Thread.sleep(time);
    }

    public void turnLeftByTime(double power, long time) throws InterruptedException {
        turnLeft(power);
        Thread.sleep(time);
    }

    public void turnRightByTime(double power, long time) throws InterruptedException {
        turnRight(power);
        Thread.sleep(time);
    }


    public void driveTillTouched(double power) {
        while (digitalTouch.getState() == true) {
            driveForward(power);
        }
    }

    public void driveTillThisClose(double power, double distance) {
        double howfar;

        while (Double.isNaN(sensorDistance.getDistance(DistanceUnit.CM)) || sensorDistance.getDistance(DistanceUnit.CM) > distance) {
            driveForward(power);
        }
    }

    public void strafeRight (double power)
    {
        motorFrontLeft.setPower(power);
        motorRearLeft.setPower(-power);
        motorFrontRight.setPower(-power);
        motorRearRight.setPower(power);
    }

    public void strafeLeft(double power) {
        motorFrontLeft.setPower(-power);
        motorRearLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorRearRight.setPower(-power);
    }

    public void strafeLeftByTime(double power, long time) throws InterruptedException {
        strafeLeft(power);
        Thread.sleep(time);
    }

    public void strafeRightByTime(double power, long time) throws InterruptedException {
        strafeRight(power);
        Thread.sleep(time);

    }

    public void TurnLeftByAngle(double power, long angle)
    {

    }

    public void TurnRightByAngle(double power, long angle)
    {

    }

    public void strafeRightTilBlue(double power)
    {

    }

    public void strafeLeftTilBlue(double power)
    {

    }

    public void strafeRightTilRed(double power)
    {

    }

    public void strafeLeftTilRed(double power)
    {

    }

    public void driveForwardTilBlue(double power)
    {

    }

    public void driveForwardTilRed(double power)
    {

    }

}
