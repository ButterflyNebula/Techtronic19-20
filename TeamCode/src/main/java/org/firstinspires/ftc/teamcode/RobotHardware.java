package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


public class RobotHardware
{
    //Chassis
    public DcMotor frontLeftWheel = null;
    public DcMotor frontRightWheel = null;
    public DcMotor backLeftWheel = null;
    public DcMotor backRightWheel = null;

    public Servo rightHook = null;
    public Servo leftHook = null;

    //Vertical Lift (Scissor Lift)
    public DcMotor rightScissor = null;
    public DcMotor leftScissor = null;
    public DigitalChannel leftScissorTouch = null;
    public DigitalChannel rightScissorTouch = null;

    public DcMotor horizontalLift = null;
    public DigitalChannel horizontalTouch = null;
    public CRServo gripperWheel = null;
    //Hand Lowering Servo
    public Servo lifter = null;

    ModernRoboticsI2cRangeSensor mrfr = null;
    ModernRoboticsI2cRangeSensor mrbr = null;
    ModernRoboticsI2cRangeSensor mrfl = null;
    ModernRoboticsI2cRangeSensor mrbl = null;

    //Webcam
    WebcamName webcam = null;


    //Adding the Hardware Map
    private HardwareMap hwMap  = null;

    public  RobotHardware(HardwareMap ahwMap)
    {
        hwMap = ahwMap;


        //Wheel motors
        frontLeftWheel = hwMap.get(DcMotor.class, "frontLeft");
        frontRightWheel = hwMap.get(DcMotor.class, "frontRight");
        backLeftWheel = hwMap.get(DcMotor.class, "backLeft");
        backRightWheel = hwMap.get(DcMotor.class, "backRight");

        frontRightWheel.setDirection(DcMotor.Direction.FORWARD);
        backRightWheel.setDirection(DcMotor.Direction.FORWARD);
        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        frontLeftWheel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        backLeftWheel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        frontRightWheel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        backRightWheel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        rightHook = hwMap.get(Servo.class, "rightHook");
        leftHook = hwMap.get(Servo.class, "leftHook");

        //Scissor
        rightScissor = hwMap.get(DcMotor.class, "rightScissor");
        rightScissor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightScissor.setDirection(DcMotor.Direction.REVERSE);
        leftScissor = hwMap.get(DcMotor.class, "leftScissor");
        leftScissor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftScissor.setDirection(DcMotor.Direction.REVERSE);

        rightScissorTouch = hwMap.get(DigitalChannel.class, "rightScissorTouch");
        leftScissorTouch = hwMap.get(DigitalChannel.class, "leftScissorTouch");


        horizontalLift = hwMap.get(DcMotor.class, "horizontalLift");
        horizontalLift.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        horizontalTouch = hwMap.get(DigitalChannel.class, "horizontalTouch");

        gripperWheel = hwMap.get(CRServo.class, "gripperWheel");

        lifter = hwMap.get(Servo.class, "lifter");

        webcam = hwMap.get(WebcamName.class, "webcam");

        mrfr = hwMap.get(ModernRoboticsI2cRangeSensor.class, "mrfr");
        mrbr = hwMap.get(ModernRoboticsI2cRangeSensor.class, "mrbr");

        mrfl = hwMap.get(ModernRoboticsI2cRangeSensor.class, "mrfl");
        mrbl = hwMap.get(ModernRoboticsI2cRangeSensor.class, "mrbl");

    }


    public HardwareMap getHwMap() {
        return hwMap;
    }
}