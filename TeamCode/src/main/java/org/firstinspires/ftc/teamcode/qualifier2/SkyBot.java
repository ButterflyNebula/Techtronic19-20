package org.firstinspires.ftc.teamcode.qualifier2;

import com.qualcomm.robotcore.hardware.HardwareMap;



/**
 * Created by Athira on 10/14/2018.
 */

public class SkyBot
{

    private static RobotHardware robotHardware = null;
    private ChassisAssembly chassisAssembly = null;
    private GripperAssembly gripperAssembly = null;
    private PlacementAssembly placementAssembly = null;
    private Navigation navigation = null;

    public void initRobot (HardwareMap hwMap)
    {
        robotHardware = new RobotHardware(hwMap);
        buildChassisAssembly();
        buildGripperAssembly();
        buildPlacementAssembly();
        buildNavigation();

    }

    public void buildChassisAssembly () {
        this.chassisAssembly = new ChassisAssembly(robotHardware);

    }

    public ChassisAssembly getChassisAssembly()
    {
        return chassisAssembly;
    }

    public void buildPlacementAssembly () {
        this.placementAssembly = new PlacementAssembly(robotHardware);

    }

    public PlacementAssembly getPlacementAssembly()
    {
        return placementAssembly;
    }


    public void buildNavigation()
    {
        this.navigation = new Navigation(robotHardware);
    }


    public void buildGripperAssembly()
    {
        this.gripperAssembly = new GripperAssembly(robotHardware);
    }

    public GripperAssembly getGripperAssembly() {return gripperAssembly;}



    public Navigation getNavigation(){return navigation;}

}
