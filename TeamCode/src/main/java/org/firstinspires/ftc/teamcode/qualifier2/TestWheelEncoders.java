package org.firstinspires.ftc.teamcode.qualifier2;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "WheelTest", group = "Test")
public class TestWheelEncoders extends LinearOpMode {

    private static final double COUNTS_PER_SCISSOR_INCH = 227.2;
    private static final double COUNTS_PER_SIDE_INCH = 50;
    private static final double COUNTS_PER_DEGREE = 8.5;
    private static final double MOTOR_SPEED = 0.8;


    // Creating a Rover robot object
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

            //*Get the position of the motor(s) being tested and print*//
            //Wheel
            telemetry.addData("FR", skyStoneBot.getChassisAssembly().getFrontRightWheelCurrentPosition());
            telemetry.addData("FL", skyStoneBot.getChassisAssembly().getFrontLeftWheelCurrentPosition());
            telemetry.addData("BR", skyStoneBot.getChassisAssembly().getBackRightWheelCurrentPosition());
            telemetry.addData("BL", skyStoneBot.getChassisAssembly().getBackLeftWheelCurrentPosition());

            telemetry.update();

            if(gamepad1.y)
            {
                skyStoneBot.getChassisAssembly().robotHardware.frontRightWheel.setPower(1);
            }
            else if(gamepad1.a)
            {
                skyStoneBot.getChassisAssembly().robotHardware.backRightWheel.setPower(1);
            }
            else if (gamepad1.dpad_up)
            {
                skyStoneBot.getChassisAssembly().robotHardware.frontLeftWheel.setPower(1);
            }
            else if(gamepad1.dpad_down)
            {
                skyStoneBot.getChassisAssembly().robotHardware.backLeftWheel.setPower(1);
            }
            else
            {
                skyStoneBot.getChassisAssembly().stopMoving();
            }
        }
    }
}
