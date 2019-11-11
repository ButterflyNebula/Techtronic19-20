package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Navigation
{
    RobotHardware robotHardware;

    protected Navigation(RobotHardware hardware)
    {
        robotHardware = hardware;
    }

    protected double mrfrDistance(){return robotHardware.mrfr.getDistance(DistanceUnit.INCH);}
    protected double mrbrDistance(){return robotHardware.mrbr.getDistance(DistanceUnit.INCH);}
    protected double mrflDistance(){return robotHardware.mrfl.getDistance(DistanceUnit.INCH);}
    protected double mrblDistance(){return robotHardware.mrbl.getDistance(DistanceUnit.INCH);}
}
