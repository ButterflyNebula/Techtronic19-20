package org.firstinspires.ftc.teamcode.qualifier2;

public class PlacementAssembly
{
    RobotHardware robotHardware;
    double slapPosition = 0.3;
    double slapperReturnPosition = 1;

    double capstonePosition = 0.65;
    double capstoneReturnPosition = 0.9;

    double doorUp = 0.7;
    double doorDown = 0;


    protected PlacementAssembly(RobotHardware hardware)
    {
        robotHardware = hardware;
    }

    public void grab (double position)
    {
        robotHardware.gripper.setPosition(position);
    }

    public void release (double position)
    {
        robotHardware.gripper.setPosition(position);
    }

    public void lifterUp (double speed){
        robotHardware.lifter.setPower(-speed);
    }

    public void lifterDown (double speed){
        robotHardware.lifter.setPower(speed);
    }

    public void lifterStay(){
        robotHardware.lifter.setPower(0);
    }

    public boolean lifterTouchState(){return robotHardware.lifterTouch.getState();}

    public void swivel (double position)
    {
        robotHardware.swivel.setPosition(position);
    }

    public void gripperSwivel (double position)
    {
        robotHardware.gripperSwivel.setPosition(position);
    }

    public double getGripperServoPosition (){
        return robotHardware.gripperSwivel.getPosition();
    }

    public double getSwivelPosition (){
        return robotHardware.swivel.getPosition();
    }

    public void slap(){robotHardware.slapper.setPosition(slapPosition);}

    public void slapperReturn(){robotHardware.slapper.setPosition(slapperReturnPosition);}

    public void slapper(double position){robotHardware.slapper.setPosition(position);}

    public void placeCapstone(){robotHardware.capstone.setPosition(capstonePosition);}

    public void capstoneReturn(){robotHardware.capstone.setPosition(capstoneReturnPosition);}

    public void capstone(double position){robotHardware.capstone.setPosition(position);}

    public void openDoor(){robotHardware.capstoneDoor.setPosition(doorUp);}

    public void closeDoor(){robotHardware.capstoneDoor.setPosition(doorDown);}

}
