package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MRSensorsTest", group = "Test")
public class TestMRSensors extends LinearOpMode
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
            telemetry.addData("MRFR", skyStoneBot.getNavigation().mrfrDistance());
            telemetry.addData("MRBR", skyStoneBot.getNavigation().mrbrDistance());
            telemetry.addData("MRFL", skyStoneBot.getNavigation().mrflDistance());
            telemetry.addData("MRBL", skyStoneBot.getNavigation().mrblDistance());
            telemetry.update();
        }
    }
}