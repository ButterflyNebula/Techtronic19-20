package org.firstinspires.ftc.teamcode.qualifier2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous
public class BlueQuarry extends LinearOpMode
{
    //Vuforia Setup
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;
    private static final String VUFORIA_KEY =
            "AVsSQB3/////AAABGehU0OjxREoznvNKEBqbvmskci8syRYfMKE0XlaGnZpw68DAZV19s7dfqc0vWrY78bAO2Ym2n1T2rDvNBOVVbMWxXIRo2c18JH6/c2fcKT1bRKxsG7bYq69+n9IHmKedY6rmTU1VOZZdtSTXh7exMsl67IAcnCZ0/ec+P+ZMpkK5v4X8d27rbEigGqqHayGe1/lG2afzgcHY7QxjJ/x5O4yGmVVs8wdzdupke19U+M8Z/x0FcYIfTAHuXcaydEL+h/w/ppcuNarD2ggo2BxdWeOGLx5GOin1yruVfvDAazPEuI0m3yEwXQNZ4e0ar2G0jDCZpAJPJcJRRVttBMwPoAvzTwySUx3qI1eNSJRAH+bk";
    //Conversions and Target Information
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;
    // Vuforia Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    //Target Found and Position variables
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    //Encoder Constants
    final double COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
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
    final double   SCISSOR_DOWN_SPEED = 0.4;
    final double INTAKE_SPEED = 0.7;
    final double SIDE_SHIFT = 6;
    final double DISTANCE_TO_GRAB_BLOCK = 24;
    final double DISTANCE_PAST_BRIDGE = 90;
    final double BACK_AWAY_FROM_BLOCK= 8;
    final double PARKING_DISTANCE = DISTANCE_PAST_BRIDGE - BACK_AWAY_FROM_BLOCK - 50;

    //Other Variables
    int blockNumber = 0;

    @Override public void runOpMode()
    {
        /**
         * INITIALIZATION
         */
        //Intialize Robot
        skyStoneBot.initRobot(hardwareMap);

        //Vuforia Setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = skyStoneBot.getNavigation().robotHardware.webcam;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //Stone Target Setup
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Camera Setup
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }
        final float CAMERA_FORWARD_DISPLACEMENT = 0 * mmPerInch;
        final float CAMERA_VERTICAL_DISPLACEMENT = 0 * mmPerInch;
        final float CAMERA_LEFT_DISPLACEMENT = 0;
        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        //Add trackables
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        //Wait for Start
        telemetry.addData("Waiting for start", "");
        telemetry.update();
        waitForStart();

        /**
         * DURING RUNTIME
         */

        //activate the target
        targetsSkyStone.activate();

        preparation();

        scanForSkyStone(stoneTarget);

        alignWithSkyStone(stoneTarget);

        //Raise the Scissor Lift and Extend the Arm.
        skyStoneBot.getGripperAssembly().lifterUp();
        sleep(500);
        encoderScissor(SCISSOR_UP_SPEED, 2, 8);
        while(opModeIsActive() && skyStoneBot.getHorizontalAssembly().horizontalTouch())
        { skyStoneBot.getHorizontalAssembly().HorizontalExtend();}
        skyStoneBot.getHorizontalAssembly().stopHorizontal();

        grabStone();

        deliverStone1();

        releaseStone();
/*
        faceNextStone();

        grabStone();

        deliverStone2();

        releaseStone();

 */

        //Park
        encoderDrive(WHEEL_SPEED, -PARKING_DISTANCE, 8);

        targetsSkyStone.deactivate();
    }

    public void preparation()
    {
        runtime.reset();

        //Move to the Right
        encoderSide(WHEEL_SPEED, 24, 8);


        //Move forward in preperation to Scan for skystone target
        encoderDrive(WHEEL_SPEED, 15, 10);
        skyStoneBot.getChassisAssembly().stopMoving();
    }

    public void scanForSkyStone(VuforiaTrackable stoneTarget)
    {
        runtime.reset();
        targetVisible = false;
        while (targetVisible == false && opModeIsActive()) {
            //Use Encoder
            encoderSide(WHEEL_SPEED, -SIDE_SHIFT, 5);

            ElapsedTime senseTime = new ElapsedTime();
            while (targetVisible == false && opModeIsActive() && senseTime.seconds() < 0.25)
            {
                telemetry.addData("Sensing...", "");
                telemetry.update();
                if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
                    targetVisible = true;
                    telemetry.addData("Visible Target", stoneTarget.getName());
                    telemetry.update();

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }

                    break;
                }
            }

            telemetry.addData("Target Found", targetVisible);
            telemetry.update();
        }// end of while (targetVisible == false && opModeIsActive())
        skyStoneBot.getChassisAssembly().stopMoving();
    }

    public void alignWithSkyStone(VuforiaTrackable stoneTarget)
    {
        straightenRight(1);

        //Get the new location of the target and ensure that the target is still visible
        if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }
        else
        {
            telemetry.addData("target no longer visible!", "");
            telemetry.update();
            sleep(4000);
        }


        //Find the position of the target
        VectorF translation = lastLocation.getTranslation();
        double currentPos = skyStoneBot.getNavigation().rightDistance();

        double targetShift = translation.get(1)/mmPerInch;

        //Target Position
        double blockPos = currentPos - targetShift + 4;//plus 4 because of the the distance betweeen the camera and the sensors

        blockNumber = (int) (Math.ceil(blockPos / 8));

        //Print the Result
        telemetry.addData("currentPos", currentPos);
        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
        telemetry.addData("Block Position", blockPos);
        telemetry.addData("Block #", blockNumber);
        telemetry.addData("Shift" , Math.abs(targetShift + 4));
        telemetry.update();
        sleep(250);

        //Shift the Robot so that the arm is Centered
        if(Math.abs(targetShift + 4) > 2)
        {
            encoderSide(WHEEL_SPEED, targetShift + 4, 5);
            skyStoneBot.getChassisAssembly().stopMoving();
        }

        //Move close to the block
        double currentDistance = skyStoneBot.getNavigation().backLaserDistance();
        double distanceToDrive = DISTANCE_TO_GRAB_BLOCK - currentDistance;

        encoderDrive(WHEEL_SPEED, distanceToDrive, 5);    }

    public void grabStone()
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

    public void deliverStone1()
    {
        //Move Backwards
        encoderDrive(WHEEL_SPEED, -15, 5);

        //Move to the right to ensure that you don't hit the wall
        if(blockNumber == 2)
        {
            encoderSide(WHEEL_SPEED, -10, 5);
        }

        //Turn 90 degrees to the left to face the bridge
        encoderTurn(WHEEL_SPEED, -90, 8);

        straigtenLeft(1);

        crossBridge();


    }

    public void crossBridge()
    {
        //Measure Distance to Back
        double currentDistance = skyStoneBot.getNavigation().backLaserDistance();
        telemetry.addData("distance", currentDistance);
        telemetry.addData("To Drive", DISTANCE_PAST_BRIDGE - currentDistance);
        telemetry.update();

        encoderDrive(WHEEL_SPEED, (DISTANCE_PAST_BRIDGE - currentDistance), 10);
    }

    public void releaseStone()
    {
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 1)
        {
            skyStoneBot.getGripperAssembly().wheelOutake(INTAKE_SPEED);
        }
        skyStoneBot.getGripperAssembly().wheelOutake(0);


        encoderScissor(SCISSOR_UP_SPEED, 3, 5);
        skyStoneBot.getScissorAssembly().move(0);

        encoderDrive(WHEEL_SPEED, -BACK_AWAY_FROM_BLOCK, 8);

        lowerScissorLift();
    }

    public void faceNextStone()
    {
        //Move backwards
        encoderDrive(WHEEL_SPEED, -48, 5);

        double newBlockPos;
        if(blockNumber <= 3)
        {
            newBlockPos = (8 * (blockNumber + 3)) - 4;
        }
        else if(blockNumber >= 5)
        {
            newBlockPos = (8 * (blockNumber - 3)) - 4;
        }
        else //the fourth block
        {
            newBlockPos = (8 * (5)) - 4; //pick up the fifth block
        }
        double currentDistance = skyStoneBot.getNavigation().backLaserDistance();
        double distanceToDrive = -4 + currentDistance - newBlockPos + 9; //- 4 to centralize with the block, +9 because dstance sensor is 9 in from the center of robot
        telemetry.addData("current distance", currentDistance);
        telemetry.addData("New Block Pos", newBlockPos);
        telemetry.addData("Distance to Drive", distanceToDrive);
        telemetry.update();

        encoderDrive(WHEEL_SPEED, -distanceToDrive, 8);

        straigtenLeft(1);

        encoderTurn(WHEEL_SPEED, 90, 6);

        encoderScissor(SCISSOR_UP_SPEED, 2, 5);

        //Move close to the block
        currentDistance = skyStoneBot.getNavigation().backLaserDistance();
        distanceToDrive = DISTANCE_TO_GRAB_BLOCK - currentDistance;

        encoderDrive(WHEEL_SPEED, distanceToDrive, 5);
    }

    public void deliverStone2()
    {
        //Move Backward
        encoderDrive(WHEEL_SPEED, -10, 5);

        //Turn to face the bridge
        encoderTurn(WHEEL_SPEED, -90, 8);

        straigtenLeft(1);

        crossBridge();
    }


    public void straightenRight(double timeOut)
    {
        double angle = skyStoneBot.getNavigation().rightAngle();
        ElapsedTime senseTime = new ElapsedTime();
        while(Math.abs(angle) > 15 && opModeIsActive() && senseTime.seconds() < timeOut)
        {
            angle = skyStoneBot.getNavigation().rightAngle();
        }
        if(angle > 360)
        {
            angle = 0;
        }

        telemetry.addData("Angle", angle);
        telemetry.addData("MRFR", skyStoneBot.getNavigation().mrfrDistance());
        telemetry.addData("MRBR", skyStoneBot.getNavigation().mrbrDistance());
        telemetry.update();
        sleep(500);

        encoderTurn(WHEEL_SPEED, angle, 3);
    }


    public void straigtenLeft(double timeOut)
    {
        double angle = -skyStoneBot.getNavigation().leftAngle();
        ElapsedTime senseTime = new ElapsedTime();
        while(Math.abs(angle) > 15 && opModeIsActive() && senseTime.seconds() < timeOut)
        {
            angle = -skyStoneBot.getNavigation().leftAngle();
        }
        if(angle > 360)
        {
            angle = 0;
        }
        telemetry.addData("Angle", angle);
        telemetry.addData("MRFL", skyStoneBot.getNavigation().mrflDistance());
        telemetry.addData("MRBL", skyStoneBot.getNavigation().mrblDistance());
        telemetry.update();
        sleep(500);

        encoderTurn(WHEEL_SPEED, angle, 5);
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
