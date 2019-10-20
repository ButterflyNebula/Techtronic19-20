package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;



/**
 * Created by Athira on 10/14/2018.
 */

public class SkyBot
{

    private static RobotHardware robotHardware = null;
    private ChassisAssembly chassisAssembly = null;
    private ScissorAssembly scissorAssembly = null;


    public void initRobot (HardwareMap hwMap)
    {
        robotHardware = new RobotHardware(hwMap);
        buildChassisAssembly();
        buildScissorAssembly();

    }

    public void buildChassisAssembly () {
        this.chassisAssembly = new ChassisAssembly(robotHardware);

    }

    public ChassisAssembly getChassisAssembly()
    {
        return chassisAssembly;
    }

    public void buildScissorAssembly () {
        this.scissorAssembly = new ScissorAssembly(robotHardware);

    }

    public ScissorAssembly getScissorAssembly()
    {
        return scissorAssembly;
    }

}
