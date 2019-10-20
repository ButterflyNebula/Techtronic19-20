package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpMode", group = "Qualifier")
public class TeleOpMode extends LinearOpMode {
    private static double WHEEL_SPEED = 1.0;
    private static double SCISSOR_SPEED = 1.0;

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

        while (opModeIsActive()) {

            /**
             * GAME PAD 1
             */
            //Controls
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            double sideRight = gamepad1.right_trigger;
            double sideLeft = gamepad1.left_trigger;

            double scissor = gamepad1.right_stick_y;


            //Movement
            if (drive > 0) {
                skyStoneBot.getChassisAssembly().moveForward(-WHEEL_SPEED * drive);
            }
            //backwards
            else if (drive < 0) {
                skyStoneBot.getChassisAssembly().moveBackwards(WHEEL_SPEED * drive);
            }
            //turn right
            else if (turn > 0) {
                skyStoneBot.getChassisAssembly().turnRight(WHEEL_SPEED * turn);
            }
            //turn left
            else if (turn < 0) {
                skyStoneBot.getChassisAssembly().turnLeft(WHEEL_SPEED * -turn);
            }
            //side right
            else if (sideRight > 0) {
                skyStoneBot.getChassisAssembly().moveRight(-WHEEL_SPEED * sideRight);
            }
            //side left
            else if (sideLeft > 0) {
                skyStoneBot.getChassisAssembly().moveLeft(-WHEEL_SPEED * sideLeft);
            }
            //stop moving
            else {
                skyStoneBot.getChassisAssembly().stopMoving();
            }

            //Scissor
            if(scissor != 0)
            {
                skyStoneBot.getScissorAssembly().move(SCISSOR_SPEED * scissor);
            }
            else
            {
                skyStoneBot.getScissorAssembly().move(0);
            }

        }
    }
}