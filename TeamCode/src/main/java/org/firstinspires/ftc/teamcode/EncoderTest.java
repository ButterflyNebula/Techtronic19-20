package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "EncoderTest", group = "Test")
public class EncoderTest extends LinearOpMode {

    private static final double COUNTS_PER_INCH = 227.2;
    private static final double COUNTS_PER_DEGREE = 14.7;
    private static final double MOTOR_SPEED = 0.2;


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
            //Gamepad Controls

            //Get the position of the motor(s) being tested and print
            /*
            telemetry.addData("FR", skyStoneBot.getChassisAssembly().getFrontRightWheelCurrentPosition());
            telemetry.addData("FL", skyStoneBot.getChassisAssembly().getFrontLeftWheelCurrentPosition());
            telemetry.addData("BR", skyStoneBot.getChassisAssembly().getBackRightWheelCurrentPosition());
            telemetry.addData("BL", skyStoneBot.getChassisAssembly().getBackLeftWheelCurrentPosition());


             */
            telemetry.addData("Right Scissor", skyStoneBot.getScissorAssembly().getRightPosition());
            telemetry.addData("Left Scissor", skyStoneBot.getScissorAssembly().getLeftPosition());

            telemetry.update();

            //Move the motor
            if (gamepad1.x)
            {
                encoderScissor(MOTOR_SPEED, 3, 8);
                //skyStoneBot.getChassisAssembly().turnRight(MOTOR_SPEED);
            }
            else if(gamepad1.y)
            {
                encoderScissor(MOTOR_SPEED, -30, 8);
                skyStoneBot.getScissorAssembly().move(0);
                telemetry.addData("Reached Bottom, going to align", "");
                telemetry.addData("Right", skyStoneBot.getScissorAssembly().rightTouch());
                telemetry.addData("Left", skyStoneBot.getScissorAssembly().leftTouch());
                telemetry.update();
                sleep(3000);

                while(opModeIsActive() && skyStoneBot.getScissorAssembly().rightTouch())
                {
                    skyStoneBot.getScissorAssembly().moveRight(-MOTOR_SPEED);
                }
                skyStoneBot.getScissorAssembly().move(0);
                while(opModeIsActive() && skyStoneBot.getScissorAssembly().leftTouch())
                {
                    skyStoneBot.getScissorAssembly().moveLeft(-MOTOR_SPEED);
                }
                skyStoneBot.getScissorAssembly().move(0);
            }
            else {
                skyStoneBot.getScissorAssembly().move(0);
                // skyStoneBot.getChassisAssembly().stopMoving();
            }

            if(gamepad1.dpad_up)
            {
                skyStoneBot.getScissorAssembly().moveRight(0.2);
            }
            else
            {
                skyStoneBot.getScissorAssembly().move(0);
            }

        }
    }

    public void encoderWheel(double speed, double degrees, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            newBackLeftTarget = skyStoneBot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
            newBackRightTarget = skyStoneBot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);
            newFrontLeftTarget = skyStoneBot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int) (degrees * COUNTS_PER_DEGREE);
            newFrontRightTarget = skyStoneBot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int) (-degrees * COUNTS_PER_DEGREE);

            skyStoneBot.getChassisAssembly().setBackLeftWheelTargetPosition(newBackLeftTarget);
            skyStoneBot.getChassisAssembly().setBackRightWheelTargetPosition(newBackRightTarget);
            skyStoneBot.getChassisAssembly().setFrontLeftWheelTargetPosition(newFrontLeftTarget);
            skyStoneBot.getChassisAssembly().setFrontRightWeelTargetPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            skyStoneBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            skyStoneBot.getChassisAssembly().setBackLeftWheelPower(Math.abs(speed));
            skyStoneBot.getChassisAssembly().setBackRightWheelPower(Math.abs(speed));
            skyStoneBot.getChassisAssembly().setFrontLeftWheelPower(Math.abs(speed));
            skyStoneBot.getChassisAssembly().setFrontRightWheelPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (skyStoneBot.getChassisAssembly().isBackLeftWheelBusy() && skyStoneBot.getChassisAssembly().isBackRightWheelBusy() &&
                            skyStoneBot.getChassisAssembly().isFrontLeftWheelBusy() && skyStoneBot.getChassisAssembly().isFrontRightWheelBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget, newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d : %7d : %7d",
                        skyStoneBot.getChassisAssembly().getBackLeftWheelCurrentPosition(),
                        skyStoneBot.getChassisAssembly().getBackRightWheelCurrentPosition(),
                        skyStoneBot.getChassisAssembly().getFrontLeftWheelCurrentPosition(),
                        skyStoneBot.getChassisAssembly().getFrontRightWheelCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            skyStoneBot.getChassisAssembly().stopMoving();

            // Turn off RUN_TO_POSITION
            skyStoneBot.getChassisAssembly().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        sleep(250);
    }//end of encoderWheel

    public void encoderScissor(double speed, double inches, double timeoutS) {

        telemetry.addData("In encoder Scissor", inches);
        telemetry.update();
        sleep(4000);

        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = skyStoneBot.getScissorAssembly().getLeftPosition() + (int) (inches * -COUNTS_PER_INCH);
            newRightTarget = skyStoneBot.getScissorAssembly().getRightPosition() + (int) (inches * -COUNTS_PER_INCH);
            skyStoneBot.getScissorAssembly().setLeftPosition(newLeftTarget);
            skyStoneBot.getScissorAssembly().setRightPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            skyStoneBot.getScissorAssembly().setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            skyStoneBot.getScissorAssembly().move(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            if(inches < 0)
            {
                while (opModeIsActive() && skyStoneBot.getScissorAssembly().leftTouch() && skyStoneBot.getScissorAssembly().rightTouch() &&
                        (runtime.seconds() < timeoutS) &&
                        (skyStoneBot.getScissorAssembly().isLeftBusy() && skyStoneBot.getScissorAssembly().isRightBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            skyStoneBot.getScissorAssembly().getLeftPosition(),
                            skyStoneBot.getScissorAssembly().getRightPosition());
                    telemetry.update();
                }
            }
            else
            {
                while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                        (skyStoneBot.getScissorAssembly().isLeftBusy() && skyStoneBot.getScissorAssembly().isRightBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            skyStoneBot.getScissorAssembly().getLeftPosition(),
                            skyStoneBot.getScissorAssembly().getRightPosition());
                    telemetry.update();
                }
            }

            // Stop all motion;
            skyStoneBot.getScissorAssembly().move(0);

            // Turn off RUN_TO_POSITION
            skyStoneBot.getScissorAssembly().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

        }
    }
}
