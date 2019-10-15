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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name = "BasicAuto", group = "12806")

public class BasicAuto extends LinearOpMode {

    Robokenbot robot   = new Robokenbot();
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     SCALE_FACTOR = 75.0/76.0; //  if drive speed = .2 use 75.0/76.0; if drive_speed = .1, use 1.0; if drive_speed = .3, use 75.0/77.0 note that .3 has hard time braking
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (SCALE_FACTOR * COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.2;
    static final double     TURN_SPEED              = 0.1;



    double DRIVE_POWER = 1;

    public void runOpMode() throws InterruptedException {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.motorRearRight.setDirection(DcMotor.Direction.FORWARD);
        robot.motorRearLeft.setDirection(DcMotor.Direction.FORWARD);




        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.motorFrontLeft.getCurrentPosition(),
                robot.motorFrontRight.getCurrentPosition(),
                robot.motorRearLeft.getCurrentPosition(),
                robot.motorRearRight.getCurrentPosition());
        telemetry.update();


        waitForStart();
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        robot.claw.setPosition(1.0);
        encoderDrive(DRIVE_SPEED,  75,  75, 50.0);  // S1: Forward 24 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //robot.Claw.setPosition(1.0);
        //encoderDrive(DRIVE_SPEED, 32, 32, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout


        telemetry.addData("Path", "Complete");
        telemetry.update();
        /*
        StrafeRightTime(DRIVE_POWER, 4000);

        DriveForwardTime(DRIVE_POWER, 2000);

        // lower and grab foundation = need real claw cdde
        robot.Claw.setPosition(0.0);
        sleep(500);
        robot.Claw.setPosition(1.0);
        DriveForwardTime(-DRIVE_POWER, 2000);
        StopDriving();

         */
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;

        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.motorRearLeft.setDirection(DcMotor.Direction.REVERSE);


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = (robot.motorFrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
            newFrontRightTarget = (robot.motorFrontRight.getCurrentPosition() - (int)(rightInches * COUNTS_PER_INCH));
            newRearLeftTarget = -(robot.motorRearLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
            newRearRightTarget = -(robot.motorRearRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH));

            robot.motorFrontLeft.setTargetPosition(newFrontLeftTarget);
            robot.motorFrontRight.setTargetPosition(newFrontRightTarget);
            robot.motorRearLeft.setTargetPosition(newRearLeftTarget);
            robot.motorRearRight.setTargetPosition(newRearRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();

            robot.motorFrontLeft.setPower(Math.abs(speed));
            robot.motorFrontRight.setPower(Math.abs(speed));
            robot.motorRearLeft.setPower(Math.abs(speed));
            robot.motorRearRight.setPower(-Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
               (robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy() && robot.motorRearLeft.isBusy() && robot.motorRearRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d : %d : %d", newFrontLeftTarget,  newFrontRightTarget, newRearLeftTarget, newRearRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d : %7d : %7d",
                        robot.motorFrontLeft.getCurrentPosition(),
                        robot.motorFrontRight.getCurrentPosition(),
                        robot.motorRearLeft.getCurrentPosition(),
                        robot.motorRearRight.getCurrentPosition());
                telemetry.update();

            }

            // Stop all motion;
            robot.motorFrontLeft.setPower(0);
            robot.motorFrontRight.setPower(0);
            robot.motorRearLeft.setPower(0);
            robot.motorRearRight.setPower(0);

            // Turn off RUN_TO_POSITION
           robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           robot.motorRearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           robot.motorRearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }

    public void driveForward(double power) {
        robot.motorFrontLeft.setPower(power);
       robot.motorFrontRight.setPower(power);
    }

    public void stopDriving() {
        driveForward(0);
    }

    public void turnLeft(double power) {
        robot.motorFrontLeft.setPower(-power);
        robot.motorFrontRight.setPower(power);
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
        while (robot.digitalTouch.getState() == true) {
            driveForward(power);
        }
    }

    public void driveTillThisClose(double power, double distance) {
        double howfar;

        while (Double.isNaN(robot.sensorDistance.getDistance(DistanceUnit.CM)) || robot.sensorDistance.getDistance(DistanceUnit.CM) > distance) {
            driveForward(power);
        }
    }

    public void strafeRight(double power)
    {
        robot.motorFrontLeft.setPower(-power);
        robot.motorRearLeft.setPower(power);
        robot.motorFrontRight.setPower(power);
        robot.motorRearRight.setPower(-power);
    }

    public void strafeLeft(double power) {
        robot.motorFrontLeft.setPower(power);
        robot.motorRearLeft.setPower(-power);
        robot.motorFrontRight.setPower(-power);
        robot.motorRearRight.setPower(power);
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
	
	public void driveForwardTileRed(double power)
	{
	
	}
	


}
