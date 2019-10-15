package org.firstinspires.ftc.teamcode;

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

HardwareMap hwMap = null;
private ElapsedTime period =new ElapsedTime();

public Robokenbot(){

}

public void init(HardwareMap ahwMap) {

    hwMap=ahwMap;
    motorFrontLeft = hwMap.dcMotor.get("motorFrontLeft");
    motorRearLeft = hwMap.dcMotor.get("motorRearLeft");
    motorFrontRight = hwMap.dcMotor.get("motorFrontRight");
    motorRearRight = hwMap.dcMotor.get("motorRearRight");
    //       digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");
    claw = hwMap.servo.get("servo");
    arm=hwMap.crservo.get("arm");

}







}
