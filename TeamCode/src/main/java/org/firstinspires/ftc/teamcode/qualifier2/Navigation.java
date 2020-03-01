package org.firstinspires.ftc.teamcode.qualifier2;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Navigation
{
    RobotHardware robotHardware;

    protected Navigation(RobotHardware hardware)
    {
        robotHardware = hardware;
    }


    protected double frontRightDistance(){return robotHardware.frontRightSensor.getDistance(DistanceUnit.INCH);}
    protected double backRightDistance(){return robotHardware.backRightSensor.getDistance(DistanceUnit.INCH);};

    protected double frontLeftDistance(){return robotHardware.frontLeftSensor.getDistance(DistanceUnit.INCH);}
    protected double backLeftDistance(){return robotHardware.backLeftSensor.getDistance(DistanceUnit.INCH);}

    protected double rightDistance(){return 0.5 * (frontRightDistance() + backRightDistance());};

    protected double rightAngle()
    {
        final double rightDistanceBetweenSensors = 12;

        double frontDistance = frontRightDistance() + 0.25;
        double backDistance = backRightDistance();

        if(Double.isNaN(frontDistance) || Double.isNaN((backDistance)) || frontDistance == 0 || backDistance == 0)
        {
            return 10000;
        }

        if(frontDistance > 100 || backDistance > 100)
        {
            return 10000;
        }


        double differenceInDistance = frontDistance - backDistance;

        double angle = Math.atan(differenceInDistance/rightDistanceBetweenSensors);
        angle = Math.toDegrees(angle);

        return angle;
    }

    protected double leftDistance(){return 0.5 * (frontLeftDistance() + backLeftDistance());};

    protected double leftAngle()
    {
        final double leftDistanceBetweenSensors = 10;

        double frontDistance = frontLeftDistance();
        double backDistance = backLeftDistance();

        if(Double.isNaN(frontDistance) || Double.isNaN((backDistance)) || frontDistance == 0 || backDistance == 0)
        {
            return 10000;
        }

        double differenceInDistance = frontDistance - backDistance;

        double angle = Math.atan(differenceInDistance/leftDistanceBetweenSensors);
        angle = Math.toDegrees(angle);

        return angle;
    }

    protected double backDistance() { return robotHardware.backLaser.getDistance(DistanceUnit.INCH);}
    protected double frontDistance() { return robotHardware.frontLaser.getDistance(DistanceUnit.INCH);}




}
