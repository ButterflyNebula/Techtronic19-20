package org.firstinspires.ftc.teamcode.qualifier2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "LaserDistanceTest", group = "Test")
public class TestLaserSensors extends LinearOpMode
{
    //Creating a Rover robot object
    SkyBot skyStoneBot = new SkyBot();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        skyStoneBot.initRobot(hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (opModeIsActive())
        {
            telemetry.addData("Front Laser", skyStoneBot.getNavigation().frontDistance());
            telemetry.addData("Back Laser", skyStoneBot.getNavigation().backDistance());
            telemetry.update();
        }
    }
}
