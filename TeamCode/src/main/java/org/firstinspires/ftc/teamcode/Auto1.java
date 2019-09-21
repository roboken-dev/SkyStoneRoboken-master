package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

@Autonomous(name = "Auto1")

public class Auto1 extends LinearOpMode


{
    DcMotor frontLeft = null;
    DcMotor rearLeft = null;
    DcMotor frontRight = null;
    DcMotor rearRight = null;

    public void runOpMode() throws InterruptedException
    {
        //initialize motors
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        rearLeft = hardwareMap.dcMotor.get("rearLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        rearRight = hardwareMap.dcMotor.get("rearLeft");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Wait for the game to start

        waitForStart ();


    }

    public void DriveForward(double power)
    {
        frontLeft.setPower(power);
        rearLeft.setPower(power);
        frontRight.setPower(power);
        rearRight.setPower(power);
    }
    public void StopDriving(double power)
    {
        DriveForward(0);
    }

    public void TurnLeft(double power)
    {
        frontLeft.setPower(-power);
        rearLeft.setPower(power);
        frontRight.setPower(-power);
        rearRight.setPower(power);
    }
    public void TurnRight(double power)
    {
        TurnLeft(-power);
    }

public void DriveForwardDistance(double power, int distance)
{
    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    frontLeft.setTargetPosition(distance);
    rearLeft.setTargetPosition(distance);
    frontRight.setTargetPosition(distance);
    rearRight.setTargetPosition(distance);

    frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION );
    rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION );
    frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION );
    rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION );

    DriveForward(power);

    while(frontLeft.isBusy()  && frontRight.isBusy())
        while(rearLeft.isBusy()  && rearRight.isBusy())

        {


    }
    StopDriving();
    frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    {



    }

}



}















