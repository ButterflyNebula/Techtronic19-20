package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class RedBuilding extends LinearOpMode
{
    //Encoder Constants
    final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 1.0;
    final double WHEEL_DIAMETER_INCHES = 4.0;
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    final double COUNTS_PER_SIDE_INCH = 100;
    final double COUNTS_PER_DEGREE = 16;
    final double COUNTS_PER_SCISSOR_INCH = 200;

    //Creating a Rover robot object
    SkyBot skyStoneBot = new SkyBot();

    //Time
    ElapsedTime runtime = new ElapsedTime();

    //Motion/Distance Constants
    final double WHEEL_SPEED = 1;
    final double SCISSOR_UP_SPEED = 0.6;
    final double SCISSOR_DOWN_SPEED = 0.4;
    final double INTAKE_SPEED = 0.7;
    final double SIDE_SHIFT = 6;
    final double DISTANCE_PAST_BRIDGE = 85;
    final double BACK_AWAY_FROM_BLOCK = 8;
    final double PARKING_DISTANCE = DISTANCE_PAST_BRIDGE - BACK_AWAY_FROM_BLOCK - 45;

    //Other Variables
    int blockNumber = 0;

    @Override
    public void runOpMode()
    {
        /**
         * INITIALIZATION
         */
        //Intialize Robot
        skyStoneBot.initRobot(hardwareMap);


        //Wait for Start
        telemetry.addData("Waiting for start", "");
        telemetry.update();
        waitForStart();

        //Centralize with the foundation
        encoderSide(WHEEL_SPEED, -15, 5);

        //Use the wall to straighten
        encoderDrive(WHEEL_SPEED/2, 4, 2);

        //Drive forward to the foundation
        encoderDrive(WHEEL_SPEED, -28, 8);
        encoderDrive(WHEEL_SPEED / 2, -4, 8);

        //Hook onto the foundation
        skyStoneBot.getChassisAssembly().closeHook();
        sleep(1000);

        //Pull the foundation back
        encoderDrive(WHEEL_SPEED, 28, 8);
        encoderDrive(WHEEL_SPEED / 2, 6, 8);

        //Turn the foundation
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2)
        {
            skyStoneBot.getChassisAssembly().moveRight(WHEEL_SPEED);
        }
        skyStoneBot.getChassisAssembly().stopMoving();

        //Let go of the foundation
        skyStoneBot.getChassisAssembly().openHook();
        sleep(1000);

        //Push the foundation into the building site
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 3)
        {
            skyStoneBot.getChassisAssembly().moveBackwards(WHEEL_SPEED / 2);
        }
        skyStoneBot.getChassisAssembly().stopMoving();

        //Make sure that it won't hit the main bridge
        if(skyStoneBot.getNavigation().leftDistance() > 24)
        {
            encoderDrive(WHEEL_SPEED, -5, 5);
        }

        //Park
        encoderDrive(WHEEL_SPEED, 45, 8);
    }



    public void encoderDrive(double speed, double inches, double timeoutS)
    {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            skyStoneBot.getChassisAssembly().changeToEncoderMode();

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = skyStoneBot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newBackRightTarget = skyStoneBot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontLeftTarget = skyStoneBot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontRightTarget = skyStoneBot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int)(inches * COUNTS_PER_INCH);



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
                telemetry.addData("Path1",  "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget,  newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d : %7d : %7d",
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

    }//end of encoderDrive

    public void encoderSide(double speed, double inches, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            newBackLeftTarget = skyStoneBot.getChassisAssembly().getBackLeftWheelCurrentPosition() + (int) (inches * COUNTS_PER_SIDE_INCH);
            newBackRightTarget = skyStoneBot.getChassisAssembly().getBackRightWheelCurrentPosition() + (int) (-inches * COUNTS_PER_SIDE_INCH);
            newFrontLeftTarget = skyStoneBot.getChassisAssembly().getFrontLeftWheelCurrentPosition() + (int) (-inches * COUNTS_PER_SIDE_INCH);
            newFrontRightTarget = skyStoneBot.getChassisAssembly().getFrontRightWheelCurrentPosition() + (int) (inches * COUNTS_PER_SIDE_INCH);

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
    }//end of encoderSide

    public void encoderTurn(double speed, double degrees, double timeoutS) {
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
    }//end of encoderTurn

    public void encoderScissor(double speed, double inches, double timeoutS) {

        telemetry.addData("In Encoder Scissor", inches);
        telemetry.update();

        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = skyStoneBot.getScissorAssembly().getLeftPosition() + (int) (inches * -COUNTS_PER_SCISSOR_INCH);
            newRightTarget = skyStoneBot.getScissorAssembly().getRightPosition() + (int) (inches * -COUNTS_PER_SCISSOR_INCH);
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
    }//end of encoderScissor
}
