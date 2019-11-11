package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpModeS", group = "Scrimmage")
public class TeleOpMode extends LinearOpMode {
    private static double   WHEEL_SPEED = 1.0;
    private static double   SCISSOR_SPEED = 0.5;
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     COUNTS_PER_INCH         = 1185.33;
    private int position = 0;
    int stoneNumber = 0;
    double currentGripperHeight = 0;
    double grabHeight = 0.25;


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


            //Gamepad 1 Controls
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            double sideRight = gamepad1.right_trigger;
            double sideLeft = gamepad1.left_trigger;
            boolean autoGrab = gamepad1.x;
            boolean autoPlace = gamepad1.b;
            boolean startPos = gamepad1.a;

            //Gamepad 2 Controls
            boolean openHook = gamepad2.b;
            boolean closeHook = gamepad2.x;
            float scissorMotion = gamepad2.left_stick_y;
            boolean gripperOpen = gamepad2.dpad_left;
            boolean gripperClose = gamepad2.dpad_right;
            boolean lifterDown= gamepad2.dpad_down;
            boolean lifterUp = gamepad2.dpad_up;
            float horizontalMotion = gamepad2.right_stick_x;
            boolean grab = gamepad1.dpad_right;
            boolean release = gamepad1.dpad_left;

            if (grab)
            {
                skyStoneBot.getGripperAssembly().wheelIntake(-0.7);
            }
            else if (release)
            {
                skyStoneBot.getGripperAssembly().wheelIntake(0.7);
            }
            else
            {
                skyStoneBot.getGripperAssembly().wheelIntake(0);
            }
            //GamePad 1 Controls
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
                skyStoneBot.getChassisAssembly().moveRight(WHEEL_SPEED * sideRight);
            }
            //side left
            else if (sideLeft > 0) {
                skyStoneBot.getChassisAssembly().moveLeft(WHEEL_SPEED * sideLeft);
            }
            //stop moving
            else {
                skyStoneBot.getChassisAssembly().stopMoving();
            }



            if(startPos)
            {
                skyStoneBot.getGripperAssembly().lifterUp();
                sleep(750);

                encoderScissor(1, -1, 7);
                skyStoneBot.getScissorAssembly().move(0);



            }

            //Gamepad2 Controls
            //Hooks
            if(openHook)
            {
                skyStoneBot.getChassisAssembly().openHook();
                sleep(250);
            }

            if(closeHook)
            {
                skyStoneBot.getChassisAssembly().closeHook();
                sleep(250);
            }

            //Scissor Motion
            if (scissorMotion > 0)
            {
                encoderScissor(1, 0.25,8);
                skyStoneBot.getScissorAssembly().move(0);
                sleep(250);
            }
            else if (scissorMotion < 0)
            {
                encoderScissor(0.25, -0.25,8);
                skyStoneBot.getScissorAssembly().move(0);
                sleep(250);
            }
            else
            {
                skyStoneBot.getScissorAssembly().move(0);
            }

            //Lifter motion
            if(lifterUp)
            {
                skyStoneBot.getGripperAssembly().lifterUp();
                sleep(250);
            }
            if(lifterDown)
            {
                skyStoneBot.getGripperAssembly().lifterDown();
                sleep(250);
            }

            //horizontal motion
            if (horizontalMotion > 0)
            {
                skyStoneBot.getHorizontalAssembly().HorizontalExtend();
            }
            else if (horizontalMotion < 0)
            {
                skyStoneBot.getHorizontalAssembly().HorizonatlRetract();
            }
            else
            {
                skyStoneBot.getHorizontalAssembly().stopHorizontal();
            }
        }
    }

    public void encoderScissor(double speed,
                             double inches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget =  skyStoneBot.getScissorAssembly().robotHardware.leftScissor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightTarget =  skyStoneBot.getScissorAssembly().robotHardware.rightScissor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            skyStoneBot.getScissorAssembly().robotHardware.leftScissor.setTargetPosition(newLeftTarget);
            skyStoneBot.getScissorAssembly().robotHardware.rightScissor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            skyStoneBot.getScissorAssembly().robotHardware.leftScissor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            skyStoneBot.getScissorAssembly().robotHardware.rightScissor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            skyStoneBot.getScissorAssembly().robotHardware.leftScissor.setPower(Math.abs(speed));
            skyStoneBot.getScissorAssembly().robotHardware.rightScissor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ( skyStoneBot.getScissorAssembly().robotHardware.leftScissor.isBusy() &&  skyStoneBot.getScissorAssembly().robotHardware.rightScissor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        skyStoneBot.getScissorAssembly().robotHardware.leftScissor.getCurrentPosition(),
                        skyStoneBot.getScissorAssembly().robotHardware.rightScissor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            skyStoneBot.getScissorAssembly().robotHardware.leftScissor.setPower(0);
            skyStoneBot.getScissorAssembly().robotHardware.rightScissor.setPower(0);

            // Turn off RUN_TO_POSITION
            skyStoneBot.getScissorAssembly().robotHardware.leftScissor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            skyStoneBot.getScissorAssembly().robotHardware.rightScissor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

            currentGripperHeight += -(inches);
        }
    }
}