package org.firstinspires.ftc.teamcode.qualifier1;

import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Created by Athira on 10/14/2018.
 */

public class SkyBot
{

    private static RobotHardware robotHardware = null;
    private ChassisAssembly chassisAssembly = null;
    private ScissorAssembly scissorAssembly = null;
    private GripperAssembly gripperAssembly = null;
    private HorizontalAssembly horizontalAssembly = null;
    private Navigation navigation = null;

    public void initRobot (HardwareMap hwMap)
    {
        robotHardware = new RobotHardware(hwMap);
        buildChassisAssembly();
        buildScissorAssembly();
        buildGripperAssembly();
        buildHorizontalAssembly();
        buildNavigation();
    }

    public void buildChassisAssembly () {
        this.chassisAssembly = new ChassisAssembly(robotHardware);

    }

    public ChassisAssembly getChassisAssembly()
    {
        return chassisAssembly;
    }

    public void buildScissorAssembly ()
    {
        this.scissorAssembly = new ScissorAssembly(robotHardware);

    }

    public void buildNavigation()
    {
        this.navigation = new Navigation(robotHardware);
    }

    public ScissorAssembly getScissorAssembly()
    {
        return scissorAssembly;
    }

    public void buildGripperAssembly()
    {
        this.gripperAssembly = new GripperAssembly(robotHardware);
    }

    public GripperAssembly getGripperAssembly() {return gripperAssembly;}

    public void buildHorizontalAssembly()
    {
        this.horizontalAssembly = new HorizontalAssembly(robotHardware);
    }

    public HorizontalAssembly getHorizontalAssembly() {return horizontalAssembly;}

    public Navigation getNavigation(){return navigation;}

}
