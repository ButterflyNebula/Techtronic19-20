package org.firstinspires.ftc.teamcode.qualifier1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class QuarryAuto extends LinearOpMode {
    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;


    private static final String VUFORIA_KEY =
            "AVsSQB3/////AAABGehU0OjxREoznvNKEBqbvmskci8syRYfMKE0XlaGnZpw68DAZV19s7dfqc0vWrY78bAO2Ym2n1T2rDvNBOVVbMWxXIRo2c18JH6/c2fcKT1bRKxsG7bYq69+n9IHmKedY6rmTU1VOZZdtSTXh7exMsl67IAcnCZ0/ec+P+ZMpkK5v4X8d27rbEigGqqHayGe1/lG2afzgcHY7QxjJ/x5O4yGmVVs8wdzdupke19U+M8Z/x0FcYIfTAHuXcaydEL+h/w/ppcuNarD2ggo2BxdWeOGLx5GOin1yruVfvDAazPEuI0m3yEwXQNZ4e0ar2G0jDCZpAJPJcJRRVttBMwPoAvzTwySUx3qI1eNSJRAH+bk";


    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

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


    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    @Override public void runOpMode() {

        skyStoneBot.initRobot(hardwareMap);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;


        parameters.cameraName = skyStoneBot.getNavigation().robotHardware.webcam;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);


        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);


        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        final float CAMERA_FORWARD_DISPLACEMENT = 0 * mmPerInch;
        final float CAMERA_VERTICAL_DISPLACEMENT = 0 * mmPerInch;
        final float CAMERA_LEFT_DISPLACEMENT = 0;

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        telemetry.addData("Waiting for start", "");
        telemetry.update();
        waitForStart();


        targetsSkyStone.activate();

        runtime.reset();

        //Move to the Right
        encoderSide(WHEEL_SPEED, 24, 8);


        //Move forward in preperation to Scan for skystone target
        encoderDrive(WHEEL_SPEED, 15, 10);
        skyStoneBot.getChassisAssembly().stopMoving();

        runtime.reset();
        while (targetVisible == false && opModeIsActive()) {
            //Use Encoder
            encoderSide(WHEEL_SPEED, -SIDE_SHIFT, 5);

            ElapsedTime senseTime = new ElapsedTime();
            while (targetVisible == false && opModeIsActive() && senseTime.seconds() < 1)
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


        //Straighten the Robot
        double angle = skyStoneBot.getNavigation().rightAngle();
        if(angle > 360)
        {
            angle = 0;
        }
        telemetry.addData("Angle", angle);
        telemetry.addData("MRFR", skyStoneBot.getNavigation().mrfrDistance());
        telemetry.addData("MRBR", skyStoneBot.getNavigation().mrbrDistance());
        telemetry.update();
        sleep(1000);

        encoderTurn(WHEEL_SPEED, angle, 5);


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

        //Block Position
        double blockPos = currentPos - (translation.get(1)/mmPerInch) + 4;//plus 4 because of the the distance betweeen the camera and the sensors

        double blockNumber = Math.ceil(blockPos / 8);


        //Print the Result
        telemetry.addData("currentPos", currentPos);
        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
        telemetry.addData("Block Position", blockPos);
        telemetry.addData("Block #", blockNumber);
        telemetry.addData("Shift" , Math.abs((translation.get(1)/mmPerInch) + 4));
        telemetry.update();
        sleep(1000);

        //Shift the Robot so that the arm is Centered
        if(Math.abs((translation.get(1)/mmPerInch) + 4) > 2)
        {
            encoderSide(WHEEL_SPEED, (translation.get(1)/mmPerInch) + 4, 5);
            skyStoneBot.getChassisAssembly().stopMoving();
        }

        //Move close to the block
        encoderDrive(WHEEL_SPEED, 8, 5);


        //Raise the Scissor Lift and Extend the Arm.
        skyStoneBot.getGripperAssembly().lifterUp();
        sleep(500);

        encoderScissor(SCISSOR_UP_SPEED, 5, 8);
        skyStoneBot.getScissorAssembly().move(0);

        while(opModeIsActive() && skyStoneBot.getHorizontalAssembly().horizontalTouch())
        {
            skyStoneBot.getHorizontalAssembly().HorizontalExtend();
        }
        skyStoneBot.getHorizontalAssembly().stopHorizontal();

        //Grab the Block
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


        //Lift the Scissor Up
        //encoderScissor(SCISSOR_UP_SPEED, 5, 8);
        //skyStoneBot.getScissorAssembly().move(0);

        //Move Backwards
        encoderDrive(WHEEL_SPEED, -15, 5);

        //Move to the left to ensure that you don't hit the wall
        encoderSide(WHEEL_SPEED, -10, 5);

        //Turn 90 degrees to the left to face the bridge
        encoderTurn(WHEEL_SPEED, -90, 8);

        //Use the two left sensors to find the angle
        double adjustAngle = -skyStoneBot.getNavigation().leftAngle();
        runtime.reset();
        while(Math.abs(adjustAngle) > 15 && opModeIsActive() && runtime.seconds() < 2)
        {
            adjustAngle = -skyStoneBot.getNavigation().leftAngle();
        }
        if(adjustAngle > 360)
        {
            adjustAngle = 0;
        }
        telemetry.addData("angle", adjustAngle);
        telemetry.update();

        encoderTurn(WHEEL_SPEED, adjustAngle, 5);


        //Measure Distance to Back
        double currentDistance = skyStoneBot.getNavigation().backLaserDistance();
        telemetry.addData("distance", currentDistance);
        telemetry.addData("To Drive", 90 - currentDistance);
        telemetry.update();
        sleep(1000);

        encoderDrive(WHEEL_SPEED, (90 - currentDistance), 10);

        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 2)
        {
            skyStoneBot.getGripperAssembly().wheelOutake(INTAKE_SPEED);
        }
        skyStoneBot.getGripperAssembly().wheelOutake(0);


        encoderScissor(SCISSOR_UP_SPEED, 5, 5);
        skyStoneBot.getScissorAssembly().move(0);

        encoderDrive(WHEEL_SPEED, -10, 8);

        lowerScissorLift();

        //Move backwards
        encoderDrive(WHEEL_SPEED, -48, 5);

        double newBlockPos = (8*(blockNumber + 3)) - 4;
        currentDistance = skyStoneBot.getNavigation().backLaserDistance();
        double distanceToDrive = -4 + currentDistance - newBlockPos + 9; //- 4 to centralize with the block, +9 because dstance sensor is 9 in from the center of robot
        telemetry.addData("current distance", currentDistance);
        telemetry.addData("New Block Pos", newBlockPos);
        telemetry.addData("Distance to Drive", distanceToDrive);
        telemetry.update();
        sleep(250);

        encoderDrive(WHEEL_SPEED, -distanceToDrive, 8);

        //Use the two left sensors to find the angle
        adjustAngle = -skyStoneBot.getNavigation().leftAngle();
        runtime.reset();
        while(Math.abs(adjustAngle) > 15 && opModeIsActive() && runtime.seconds() < 2)
        {
            adjustAngle = -skyStoneBot.getNavigation().leftAngle();
        }
        if(adjustAngle > 360)
        {
            adjustAngle = 0;
        }
        telemetry.addData("angle", adjustAngle);
        telemetry.update();
        sleep(250);


        encoderTurn(WHEEL_SPEED, adjustAngle, 5);

        telemetry.addData("preparing to turn", "");
        sleep(1000);

        encoderTurn(WHEEL_SPEED, 90, 6);

        encoderScissor(SCISSOR_UP_SPEED, 5, 5);

        encoderDrive(WHEEL_SPEED, 15, 5);

        //Grab the Block
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


        //Turn to face the bridge
        encoderTurn(WHEEL_SPEED, -90, 8);

        //Straighten w/ left wall
        adjustAngle = -skyStoneBot.getNavigation().leftAngle();
        runtime.reset();
        while(Math.abs(adjustAngle) > 15 && opModeIsActive() && runtime.seconds() < 2)
        {
            adjustAngle = -skyStoneBot.getNavigation().leftAngle();
        }
        if(adjustAngle > 360)
        {
            adjustAngle = 0;
        }
        telemetry.addData("angle", adjustAngle);
        telemetry.update();
        sleep(250);

        //Move forward across the bridge
        currentDistance = skyStoneBot.getNavigation().backLaserDistance();
        distanceToDrive = 90 - currentDistance;
        telemetry.addData("distance to drive", distanceToDrive);
        telemetry.update();
        sleep(250);

        encoderDrive(WHEEL_SPEED, distanceToDrive, 6);

        //Release Bloack
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 2)
        {
            skyStoneBot.getGripperAssembly().wheelOutake(INTAKE_SPEED);
        }
        skyStoneBot.getGripperAssembly().wheelOutake(0);


        encoderScissor(SCISSOR_UP_SPEED, 5, 5);
        skyStoneBot.getScissorAssembly().move(0);

        encoderDrive(WHEEL_SPEED, -10, 8);

        lowerScissorLift();

        encoderDrive(WHEEL_SPEED, -30, 6);


        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
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