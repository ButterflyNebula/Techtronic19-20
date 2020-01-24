package org.firstinspires.ftc.teamcode.qualifier2;

public class GripperAssembly
{
    RobotHardware robotHardware;

    private static final double LIFTER_UP_POSITION = 0.1;
    private static final double LIFTER_DOWN_POSITION = 0.7;

    protected GripperAssembly(RobotHardware hardware)
    {
        robotHardware = hardware;
    }

    public void wheelIntake(double power)
    {
        robotHardware.leftIntake.setPower(power);
        robotHardware.rightIntake.setPower(power);
    }

    public void wheelOutake(double power) {
        robotHardware.leftIntake.setPower(-power);
        robotHardware.rightIntake.setPower(-power);
    }

    public void leftIntake(double power)
    {
        robotHardware.leftIntake.setPower(power);
    }

    public void leftOutake(double power)
    {
        robotHardware.leftIntake.setPower(-power);
    }

    public void rightIntake(double power)
    {
        robotHardware.rightIntake.setPower(power);
    }

    public void rightOutake(double power)
    {
        robotHardware.rightIntake.setPower(-power);
    }

    public void wheelStop() {
        robotHardware.leftIntake.setPower(0);
        robotHardware.rightIntake.setPower(0);
    }
}