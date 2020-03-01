package org.firstinspires.ftc.teamcode.qualifier2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpMode", group = "Qualifier")
public class TeleOpMode extends LinearOpMode {
    private double   WHEEL_SPEED = 1.0;
    private static double INTAKE_SPEED = 1.0;

    //Encoder Constants
    final double COUNTS_PER_MOTOR_REV    = 537.6;
    final double DRIVE_GEAR_REDUCTION = 1;
    final double WHEEL_DIAMETER_INCHES = 4.0;
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    boolean intaking = false;

    boolean capstonePlaced = false;

    //Gripper Servo Positions
    //Grabbing
    double gripperGrab = 0.5675;
    double gripperSwivelGrab = 0.9;
    double swivelGrab = 0.69;
    //Placing
    double gripperPlace = 0.2;
    double gripperSwivelPlace = 0.23;
    double swivelPlace = 0.56;


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

        skyStoneBot.getChassisAssembly().openHook();
        skyStoneBot.getPlacementAssembly().swivel(swivelPlace);
        skyStoneBot.getPlacementAssembly().gripperSwivel(gripperSwivelPlace);
        skyStoneBot.getPlacementAssembly().grab(gripperPlace);
        skyStoneBot.getPlacementAssembly().slapperReturn();

        skyStoneBot.getPlacementAssembly().capstoneReturn();
        skyStoneBot.getPlacementAssembly().closeDoor();

        double moveto = 0.56;
        double moveto2 = 0.17;

        while (opModeIsActive())
        {
            //Gamepad 1 Controls
            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            double sideRight = gamepad1.right_trigger;
            double sideLeft = gamepad1.left_trigger;

            boolean openHook = gamepad1.dpad_left;
            boolean closeHook = gamepad1.dpad_right;

            boolean gripperIntake = gamepad1.x;
            boolean gripperOutake = gamepad2.a; //changed to gamepad 2

            boolean grab = gamepad1.y;
            boolean place = gamepad1.a;
            boolean flip = gamepad1.b;

            boolean capstone = gamepad2.x;

            boolean lifterUp = gamepad1.dpad_up;
            boolean lifterDown = gamepad1.dpad_down;

            boolean rightHook = gamepad1.right_bumper;
            boolean leftHook = gamepad1.left_bumper;


            boolean scrunch = gamepad2.y;

            float tapeMeasure = gamepad2.left_stick_y;



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
         /*   else if(diagonalFrontRight > 0)
            {
                skyStoneBot.getChassisAssembly().diagonalForwardRight(WHEEL_SPEED);
            }
            else if(diagonalFrontLeft < 0)
            {
                skyStoneBot.getChassisAssembly().diagonalForwardLeft(WHEEL_SPEED);
            }
            else if(diagonalBackRight > 0)
            {
                skyStoneBot.getChassisAssembly().diagonalBackwardsRight(WHEEL_SPEED);
            }
            else if(diagonalBackLeft < 0)
            {
                skyStoneBot.getChassisAssembly().diagonalBackwardsLeft(WHEEL_SPEED);
            }
          */
            //stop moving
            else {
                skyStoneBot.getChassisAssembly().stopMoving();
            }

            //Gripper Controls
            if(gripperIntake)
            {
                skyStoneBot.getPlacementAssembly().slapperReturn();
                skyStoneBot.getGripperAssembly().wheelIntake(INTAKE_SPEED);
                intaking = true;
            }

            else if(gripperOutake)
            {
                skyStoneBot.getGripperAssembly().wheelOutake(INTAKE_SPEED);
                intaking = false;
            }
            else if(intaking == false)
            {
                skyStoneBot.getGripperAssembly().wheelStop();
            }

            //Hooks
            if(openHook)
            {
                skyStoneBot.getChassisAssembly().openHook();
                sleep(250);
                WHEEL_SPEED = 1;
            }
            else if(closeHook)
            {
                skyStoneBot.getChassisAssembly().closeHook();
                sleep(250);
                WHEEL_SPEED = 0.5;
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

            if (lifterUp){
                skyStoneBot.getPlacementAssembly().lifterUp(1.0);
            }

            else if (lifterDown && skyStoneBot.getPlacementAssembly().lifterTouchState()){
                skyStoneBot.getPlacementAssembly().lifterDown(1.0);
            }

            else {
                skyStoneBot.getPlacementAssembly().lifterStay();
            }

            if(grab)
            {
                skyStoneBot.getGripperAssembly().wheelStop();
                intaking = false;

                skyStoneBot.getPlacementAssembly().slap();
               // sleep(250);
                skyStoneBot.getPlacementAssembly().grab(gripperGrab);
            }
            else if(flip)
            {
                WHEEL_SPEED = 0.5;

                if(capstonePlaced == false)
                {
                    skyStoneBot.getPlacementAssembly().swivel(swivelGrab);
                    sleep(100);
                    skyStoneBot.getPlacementAssembly().gripperSwivel(gripperSwivelGrab);
                    skyStoneBot.getPlacementAssembly().slapperReturn();
                }
                else
                {
                    while(skyStoneBot.getPlacementAssembly().robotHardware.swivel.getPosition() < swivelGrab)
                    {
                        double currentPos = skyStoneBot.getPlacementAssembly().robotHardware.swivel.getPosition();

                        skyStoneBot.getPlacementAssembly().swivel(currentPos + 0.0005);
                    }

                    while(skyStoneBot.getPlacementAssembly().robotHardware.gripperSwivel.getPosition() < gripperSwivelGrab)
                    {
                        double currentPos = skyStoneBot.getPlacementAssembly().robotHardware.gripperSwivel.getPosition();

                        skyStoneBot.getPlacementAssembly().gripperSwivel(currentPos + 0.001);
                    }


//                    skyStoneBot.getPlacementAssembly().swivel(swivelGrab);
  //                  sleep(100);
     //               skyStoneBot.getPlacementAssembly().gripperSwivel(gripperSwivelGrab);
       //             skyStoneBot.getPlacementAssembly().slapperReturn();
                    capstonePlaced = false;
                    skyStoneBot.getPlacementAssembly().capstoneReturn();
                    skyStoneBot.getPlacementAssembly().closeDoor();
                }
            }


            else if (place){

                WHEEL_SPEED = 1;

                skyStoneBot.getPlacementAssembly().grab(gripperPlace);
                sleep(250);

               encoderDrive(0.5, 10, 5);
               // sleep(250);

                while (opModeIsActive() && skyStoneBot.getPlacementAssembly().lifterTouchState())
                {
                    skyStoneBot.getPlacementAssembly().lifterDown(1);
                }
                skyStoneBot.getPlacementAssembly().lifterStay();

                skyStoneBot.getPlacementAssembly().swivel(swivelPlace);
                skyStoneBot.getPlacementAssembly().gripperSwivel(gripperSwivelPlace);
            }

            if(capstone)
            {
                    skyStoneBot.getPlacementAssembly().openDoor();
                    sleep(250);
                    skyStoneBot.getPlacementAssembly().placeCapstone();
                    capstonePlaced = true;
            }

            if(scrunch)
            {
                skyStoneBot.getPlacementAssembly().swivel(0.2);
                skyStoneBot.getPlacementAssembly().gripperSwivel(0.9);
                skyStoneBot.getPlacementAssembly().grab(0.1);
            }

            if(tapeMeasure != 0)
            {

                skyStoneBot.getChassisAssembly().robotHardware.tapeMeasure.setPower(-tapeMeasure);
            }
            else
            {
                skyStoneBot.getChassisAssembly().robotHardware.tapeMeasure.setPower(0);
            }

        }
    }

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