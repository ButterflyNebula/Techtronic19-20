package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class HorizontalAssembly
{
    RobotHardware robotHardware;

    protected HorizontalAssembly(RobotHardware hardware)
    {
        robotHardware = hardware;
    }

    public void HorizontalExtend () {robotHardware.horizontalLift.setPower(1);}

    public void HorizonatlRetract () {robotHardware.horizontalLift.setPower(-1);}

    public void stopHorizontal () {robotHardware.horizontalLift.setPower(0);}

    public boolean horizontalTouch (){return robotHardware.horizontalTouch.getState();}

}
