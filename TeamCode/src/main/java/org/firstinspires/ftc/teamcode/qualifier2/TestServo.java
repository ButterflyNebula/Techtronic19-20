package org.firstinspires.ftc.teamcode.qualifier2;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ServoTest", group = "Test")
public class TestServo extends LinearOpMode
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
            if(gamepad1.x)
            {
                skyStoneBot.getPlacementAssembly().grab(0.1);
                sleep(1000);
                skyStoneBot.getPlacementAssembly().grab(0.5);
                sleep(1000);
                skyStoneBot.getPlacementAssembly().grab(0.9);
            }
            else if(gamepad1.y)
            {
                skyStoneBot.getPlacementAssembly().gripperSwivel(0.1);
                sleep(1000);
                skyStoneBot.getPlacementAssembly().gripperSwivel(0.5);
                sleep(1000);
                skyStoneBot.getPlacementAssembly().gripperSwivel(0.9);
            }
            else if(gamepad1.b)
            {
                skyStoneBot.getPlacementAssembly().swivel(0.1);
                sleep(1000);
                skyStoneBot.getPlacementAssembly().swivel(0.5);
                sleep(1000);
                skyStoneBot.getPlacementAssembly().swivel(0.9);
            }
            else if(gamepad1.a)
            {
                skyStoneBot.getPlacementAssembly().slapper(0.1);
                sleep(1000);
                skyStoneBot.getPlacementAssembly().slapper(0.5);
                sleep(1000);
                skyStoneBot.getPlacementAssembly().slapper(1);
            }
            else if (gamepad1.dpad_left)
            {
                skyStoneBot.getPlacementAssembly().capstone(0.65);
            }
            else if (gamepad1.dpad_right)
            {
                skyStoneBot.getPlacementAssembly().capstone(1.0);
            }
        }
    }
}