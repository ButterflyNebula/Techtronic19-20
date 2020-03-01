package org.firstinspires.ftc.teamcode.qualifier2;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "DistanceSensorsTest", group = "Test")
public class TestDistanceSensors extends LinearOpMode
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
            telemetry.addData("Front Right", skyStoneBot.getNavigation().frontRightDistance());
            telemetry.addData("Back Right", skyStoneBot.getNavigation().backRightDistance());
            telemetry.addData("Front Left", skyStoneBot.getNavigation().frontLeftDistance());
            telemetry.addData("Back Left", skyStoneBot.getNavigation().backLeftDistance());

            telemetry.addData("Front", skyStoneBot.getNavigation().frontDistance());
            telemetry.addData("Back", skyStoneBot.getNavigation().backDistance());

            telemetry.addData("Right Angle", skyStoneBot.getNavigation().rightAngle());
            telemetry.addData("Left Angle", skyStoneBot.getNavigation().leftAngle());
            telemetry.update();
        }
    }
}