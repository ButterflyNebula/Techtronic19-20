package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpQ", group = "Qualifier")
public class TeleOpMode2 extends LinearOpMode {
    private static double   WHEEL_SPEED = 1.0;
    private static double INTAKE_SPEED = 0.7;
    private static double SCISSOR_UP_SPEED = 0.6;
    private static double   SCISSOR_DOWN_SPEED = 0.4;
    final double COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 1.0;
    final double WHEEL_DIAMETER_INCHES = 4.0;
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    final double COUNTS_PER_SCISSOR_INCH = 200;



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

        skyStoneBot.getGripperAssembly().lifterUp();
        sleep(500);

        while (opModeIsActive())
        {
            //Gamepad 1 Controls
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            double sideRight = gamepad1.right_trigger;
            double sideLeft = gamepad1.left_trigger;

            boolean openHook = gamepad1.dpad_left;
            boolean closeHook = gamepad1.dpad_right;

            boolean scissorStepUp = gamepad1.dpad_up;
            boolean scissorDown = gamepad1.dpad_down;

            boolean autograb = gamepad1.x;
            boolean autoplace = gamepad1.b;

            boolean startPos = gamepad1.y;

            //Gamepad 2
            float horizontalExtend = gamepad2.right_trigger;
            float horizontalRetract = gamepad2.left_trigger;

            boolean gripperIntake = gamepad2.x;
            boolean gripperOutake = gamepad2.b;

            boolean lifterUp = gamepad2.dpad_up;
            boolean lifterDown = gamepad2.dpad_down;

            boolean rightHook = gamepad2.dpad_right;
            boolean leftHook = gamepad2.dpad_left;

            boolean scissorStepDown = gamepad2.a;


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

            //Gripper Controls
            if(gripperIntake)
            {
                skyStoneBot.getGripperAssembly().wheelIntake(INTAKE_SPEED);
            }

            else if(gripperOutake)
            {
                skyStoneBot.getGripperAssembly().wheelOutake(INTAKE_SPEED);
            }
            else
            {
                skyStoneBot.getGripperAssembly().wheelStop();
            }

            //AutoGrab
            if(autograb)
            {
                skyStoneBot.getGripperAssembly().lifterUp();
                sleep(500);
                lowerScissorLift();
                sleep(250);

                runtime.reset();
                while(opModeIsActive() && runtime.seconds() < 1)
                {
                    skyStoneBot.getGripperAssembly().wheelIntake(INTAKE_SPEED);

                    skyStoneBot.getGripperAssembly().lifterDown();

                }

                skyStoneBot.getGripperAssembly().wheelStop();
                sleep(250);

                skyStoneBot.getGripperAssembly().lifterUp();
                sleep(500);
            }

            //AutoPlace
            if(autoplace)
            {
                runtime.reset();
                while(opModeIsActive() && runtime.seconds() < 1.5)
                {
                    skyStoneBot.getGripperAssembly().wheelOutake(INTAKE_SPEED);
                }

                encoderScissor(SCISSOR_UP_SPEED, 3, 8);
                skyStoneBot.getScissorAssembly().move(0);
                skyStoneBot.getGripperAssembly().wheelStop();
            }

            if(startPos)
            {
                skyStoneBot.getGripperAssembly().lifterUp();
                sleep(500);

                encoderScissor(SCISSOR_UP_SPEED, 5, 8);
                skyStoneBot.getScissorAssembly().move(0);

                while(opModeIsActive() && skyStoneBot.getHorizontalAssembly().horizontalTouch())
                {
                    skyStoneBot.getHorizontalAssembly().HorizontalExtend();
                }
                skyStoneBot.getHorizontalAssembly().stopHorizontal();

                lowerScissorLift();
                skyStoneBot.getScissorAssembly().move(0);
            }


            //Scissor
            if(scissorStepUp)
            {
                encoderScissor(SCISSOR_UP_SPEED, 2.5, 8);
                skyStoneBot.getScissorAssembly().move(0);
            }
            else if(scissorStepDown)
            {
                encoderScissor(SCISSOR_DOWN_SPEED, -2.5, 8);
                skyStoneBot.getScissorAssembly().move(0);
            }
            else if(scissorDown)
            {
                lowerScissorLift();
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
            else if(lifterDown)
            {
                skyStoneBot.getGripperAssembly().lifterDown();
                sleep(250);
            }

            //Hooks
            if(openHook)
            {
                skyStoneBot.getChassisAssembly().openHook();
                sleep(250);
            }
            else if(closeHook)
            {
                skyStoneBot.getChassisAssembly().closeHook();
                sleep(250);
            }

            if(rightHook)
            {
                skyStoneBot.getChassisAssembly().moveRightHook();
                sleep(250);
            }
            else if(leftHook)
            {
                skyStoneBot.getChassisAssembly().moveLeftHook();
                sleep(250);
            }

            //Horizontal Slider
            if (horizontalExtend > 0 && skyStoneBot.getHorizontalAssembly().horizontalTouch())
            {
                skyStoneBot.getHorizontalAssembly().HorizontalExtend();
            }
            else if (horizontalRetract > 0)
            {
                skyStoneBot.getHorizontalAssembly().HorizonatlRetract();
            }
            else
            {
                skyStoneBot.getHorizontalAssembly().stopHorizontal();
            }
        }
    }

    public void lowerScissorLift()
    {
        encoderScissor(SCISSOR_DOWN_SPEED, -30, 8);
        skyStoneBot.getScissorAssembly().move(0);

        while(opModeIsActive() && skyStoneBot.getScissorAssembly().rightTouch())
        {
            skyStoneBot.getScissorAssembly().moveRight(-SCISSOR_DOWN_SPEED);
        }
        skyStoneBot.getScissorAssembly().move(0);
        while(opModeIsActive() && skyStoneBot.getScissorAssembly().leftTouch())
        {
            skyStoneBot.getScissorAssembly().moveLeft(-SCISSOR_DOWN_SPEED);
        }
        skyStoneBot.getScissorAssembly().move(0);
    }


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

    /**
     *ENCODER DRIVE METHOD
     * @param speed (at which the robot should move)
     * @param inches (positive is forward, negative is backwards)
     * @param timeoutS (the robot will stop moving if it after this many seconds)
     */
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
}