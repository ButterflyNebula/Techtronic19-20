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

    protected double rightDistance(){return 0.5 * (mrfrDistance() + mrbrDistance());};

    protected double rightAngle()
    {
        final double distanceBetweenSensors = 16;

        double frontDistance = mrfrDistance();
        double backDistance = mrbrDistance();


        double differenceInDistance = frontDistance - backDistance;

        double angle = Math.atan(differenceInDistance/distanceBetweenSensors);
        angle = Math.toDegrees(angle);

        return angle;
    }

    protected double mrflDistance(){return robotHardware.mrfl.getDistance(DistanceUnit.INCH);}
    protected double mrblDistance(){return robotHardware.mrbl.getDistance(DistanceUnit.INCH);}

    protected double leftDistance(){return 0.5 * (mrflDistance() + mrblDistance());};

    protected double leftAngle()
    {
        final double distanceBetweenSensors = 16;

        double frontDistance = mrflDistance();
        double backDistance = mrblDistance();


        double differenceInDistance = frontDistance - backDistance;

        double angle = Math.atan(differenceInDistance/distanceBetweenSensors);
        angle = Math.toDegrees(angle);

        return angle;
    }

}
