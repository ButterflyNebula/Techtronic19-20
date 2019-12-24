package org.firstinspires.ftc.teamcode.qualifier1;

public class GripperAssembly
{
    RobotHardware robotHardware;

    private static final double LIFTER_UP_POSITION = 0.1;
    private static final double LIFTER_DOWN_POSITION = 0.7;

    protected GripperAssembly(RobotHardware hardware)
    {
        robotHardware = hardware;
    }


    public void lifterDown ()
    {robotHardware.lifter.setPosition(LIFTER_DOWN_POSITION); }

    public void lifterUp ()
    {robotHardware.lifter.setPosition(LIFTER_UP_POSITION); }


    public void lifterMotion(double Position)
    {
        robotHardware.lifter.setPosition(Position);
    }

    public void wheelIntake(double power)
    {
        robotHardware.gripperWheel.setPower(-power);
    }

    public void wheelOutake(double power) {robotHardware.gripperWheel.setPower(power);}

    public void wheelStop() {robotHardware.gripperWheel.setPower(0);}
}