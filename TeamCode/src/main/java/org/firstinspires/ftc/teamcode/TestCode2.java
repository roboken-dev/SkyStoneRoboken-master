package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp(name = "TestTeleOp adb try")
public class TestCode2 extends LinearOpMode {

    private DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException
    {

        motor = hardwareMap.dcMotor.get("motor");
        waitForStart();

        while(opModeIsActive())
        {
            motor.setPower(gamepad1.left_stick_y);
            idle();
        }
    }

}
