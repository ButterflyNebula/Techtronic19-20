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

@Autonomous(name = "RedQuarryBridge", group = "Qualifier")
public class RedQuarryBridgeSide extends LinearOpMode {
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
    final double DRIVE_GEAR_REDUCTION = 0.5;
    final double WHEEL_DIAMETER_INCHES = 4.0;
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    final double COUNTS_PER_SIDE_INCH = 50;
    final double COUNTS_PER_DEGREE = 8.5;

    //Gripper Servo Positions
    //Grabbing
    double gripperGrab = 0.5675;
    double gripperSwivelGrab = 0.9;
    double swivelGrab = 0.69;
    //Placing
    double gripperPlace = 0.2;
    double gripperSwivelPlace = 0.23;
    double swivelPlace = 0.56;

    //Movement Constants
    final double WHEEL_SPEED = 1;
    final double SIDE_SHIFT = 6;
    final double DISTANCE_TO_GRAB_BLOCK = 30;
    final double DISTANCE_TO_BRIDGE = 75;
    final double DISTANCE_TO_PARK = 12;



    int blockNumber = 0;
    boolean defaultblock = false;

    //Creating a Rover robot object
    SkyBot skyStoneBot = new SkyBot();

    //Time
    ElapsedTime runtime = new ElapsedTime();

    @Override public void runOpMode() {
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

        if(!defaultblock) {
            alignWithSkyStone(stoneTarget);
        }

        grabStone();

        deliverStone();

        placeStone();

        parkUnderBridge();


    }

    public void preparation()
    {
        runtime.reset();

        //Move forward in preperation to Scan for skystone target
        encoderDrive(WHEEL_SPEED, 15, 10);
        skyStoneBot.getChassisAssembly().stopMoving();

        encoderSide(WHEEL_SPEED, -8, 10);

        //Servo Positions
        skyStoneBot.getChassisAssembly().openHook();
        skyStoneBot.getPlacementAssembly().swivel(swivelPlace);
        skyStoneBot.getPlacementAssembly().gripperSwivel(gripperSwivelPlace);
        skyStoneBot.getPlacementAssembly().grab(gripperPlace);
        skyStoneBot.getPlacementAssembly().slapperReturn();
    }

    public void scanForSkyStone(VuforiaTrackable stoneTarget)
    {
        boolean firstMove = true;
        int count = 0;

        runtime.reset();
        targetVisible = false;
        while (targetVisible == false && opModeIsActive() && count < 5) {
            if(!firstMove)
            {
                //Use Encoder
                encoderSide(WHEEL_SPEED, SIDE_SHIFT, 5);
            }

            firstMove = false;
            count++;

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
        if(count >= 5)
        {
            defaultblock = true;
        }
    }

    public void alignWithSkyStone(VuforiaTrackable stoneTarget)
    {
        straightenLeft(1);

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
        double blockPos = currentPos - targetShift + 14;//plus 4 because of the the distance betweeen the camera and the sensors

        blockNumber = (int) (Math.ceil(blockPos / 8));

        //Print the Result
        telemetry.addData("currentPos", currentPos);
        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
        telemetry.addData("Block Position", blockPos);
        telemetry.addData("Block #", blockNumber);
        telemetry.addData("Shift" , Math.abs(targetShift + 8));
        telemetry.update();
        sleep(250);

        //Shift the Robot so that the arm is Centered
        if(Math.abs(targetShift + 8) > 2)
        {
            encoderSide(WHEEL_SPEED, targetShift + 8, 5);
            skyStoneBot.getChassisAssembly().stopMoving();
        }

    }

    public void grabStone()
    {
        //Move close to the block
        double currentDistance = skyStoneBot.getNavigation().backDistance();
        double distanceToDrive = DISTANCE_TO_GRAB_BLOCK - currentDistance;

        encoderDrive(WHEEL_SPEED, distanceToDrive, 5);

        skyStoneBot.getGripperAssembly().wheelIntake(1);

        while (opModeIsActive() && runtime.seconds() < 2)
        {
            skyStoneBot.getChassisAssembly().moveForward(0.15);
            skyStoneBot.getGripperAssembly().wheelIntake(1);
        }

        skyStoneBot.getChassisAssembly().stopMoving();
        sleep(500);
        skyStoneBot.getGripperAssembly().wheelStop();


        skyStoneBot.getPlacementAssembly().slap();
        sleep(1000);

        skyStoneBot.getPlacementAssembly().grab(gripperGrab);
        sleep(1000);
    }

    public void deliverStone()
    {
        encoderDrive(WHEEL_SPEED, -24, 5);

        encoderTurn(WHEEL_SPEED, -90, 5);
        straightenLeft(5);

        double distanceToDrive = DISTANCE_TO_BRIDGE - skyStoneBot.getNavigation().frontDistance();

        encoderDrive(WHEEL_SPEED, -distanceToDrive, 7);


    }


    public void placeStone()
    {

        skyStoneBot.getPlacementAssembly().swivel(swivelGrab);
        sleep(500);
        skyStoneBot.getPlacementAssembly().gripperSwivel(gripperSwivelGrab);
        sleep(1000);
        skyStoneBot.getPlacementAssembly().grab(gripperPlace);
        sleep(1000);

    }

    public void parkUnderBridge()
    {
        encoderDrive(WHEEL_SPEED, DISTANCE_TO_PARK, 5);
        sleep(250);

        skyStoneBot.getPlacementAssembly().swivel(swivelPlace);
        sleep(500);
        skyStoneBot.getPlacementAssembly().gripperSwivel(gripperSwivelPlace);
    }


    public void straightenLeft(double timeOut)
    {
        double angle = -skyStoneBot.getNavigation().leftAngle();
        ElapsedTime senseTime = new ElapsedTime();
        while(Math.abs(angle) > 15 && opModeIsActive() && senseTime.seconds() < timeOut)
        {
            angle = -skyStoneBot.getNavigation().leftAngle();

            telemetry.addData("Angle", angle);
            telemetry.addData("FL", skyStoneBot.getNavigation().frontLeftDistance());
            telemetry.addData("BL", skyStoneBot.getNavigation().backLeftDistance());
            telemetry.update();
        }
        if(angle > 50)
        {
            angle = 0;
        }
        telemetry.addData("Angle", angle);
        telemetry.addData("MRFL", skyStoneBot.getNavigation().frontLeftDistance());
        telemetry.addData("MRBL", skyStoneBot.getNavigation().backLeftDistance());
        telemetry.update();

        if(Math.abs(angle) > 5)
        {
            encoderTurn(WHEEL_SPEED, angle, 5);
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
}
