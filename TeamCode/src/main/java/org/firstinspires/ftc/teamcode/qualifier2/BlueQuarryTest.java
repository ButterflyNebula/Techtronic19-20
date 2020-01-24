package org.firstinspires.ftc.teamcode.qualifier2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@Autonomous(name = "BlueQuarryTest", group = "Qualifier")
public class BlueQuarryTest extends LinearOpMode {
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


    //Creating a Rover robot object
    SkyBot skyStoneBot = new SkyBot();

    //Time
    ElapsedTime runtime = new ElapsedTime();

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    //Gripper Servo Positions
    //Grabbing
    double gripperGrab = 0.5675;
    double gripperSwivelGrab = 0.9;
    double swivelGrab = 0.69;
    //Placing
    double gripperPlace = 0.2;
    double gripperSwivelPlace = 0.23;
    double swivelPlace = 0.56;


    //Variables
    double blockNumber;
    double sideShift;


    //Constants
    final double INTAKE_SPEED = 1.0;
    final double WHEEL_SPEED = 1.0;
    final double CAMERA_TO_GRIPPER = 6;
    final double DISTANCE_TO_BRIDGE = 80;


    //Encoder Constants
    final double COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 0.5;
    final double WHEEL_DIAMETER_INCHES = 4.0;
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    final double COUNTS_PER_SIDE_INCH = 50;
    final double COUNTS_PER_DEGREE = 8.5;


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
            phoneXRotate = 90 ;
        }

        final float CAMERA_FORWARD_DISPLACEMENT  = 0 * mmPerInch;
        final float CAMERA_VERTICAL_DISPLACEMENT = 0 * mmPerInch;
        final float CAMERA_LEFT_DISPLACEMENT     = 0;

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }


        targetsSkyStone.activate();

        runtime.reset();

        while (!opModeIsActive() && !isStopRequested())
        {
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables)
            {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible())
                {
                    targetVisible = true;

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }

                    break;
                }
            }

            if (targetVisible)
            {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);


                double rightDistance = skyStoneBot.getNavigation().rightDistance() + 2;
                double blockShift = translation.get(1) / mmPerInch;
                double blockLocation = rightDistance - blockShift;
                sideShift = CAMERA_TO_GRIPPER + blockShift;
                blockNumber = (int) (Math.ceil(blockLocation / 8));
                telemetry.addData("Right Distance", rightDistance);
                telemetry.addData("BlockShift", blockShift);
                telemetry.addData("Block Location", blockLocation);
                telemetry.addData("Block Number", blockNumber);
                telemetry.addData("Side Shift", sideShift);
                telemetry.update();


            }
            else {
                telemetry.addData("Visible Target", "none");
            }

            telemetry.update();
        }

        if(targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

            double rightDistance = skyStoneBot.getNavigation().rightDistance() + 2;
            double blockShift = translation.get(1) / mmPerInch;
            double blockLocation = rightDistance - blockShift;
            sideShift = CAMERA_TO_GRIPPER + blockShift;
            blockNumber = (int) (Math.ceil(blockLocation / 8));
            telemetry.addData("Right Distance", rightDistance);
            telemetry.addData("BlockShift", blockShift);
            telemetry.addData("Block Location", blockLocation);
            telemetry.addData("Block Number", blockNumber);
            telemetry.addData("Side Shift", sideShift);
            telemetry.update();
        }
        else {
            blockNumber = 6;
            telemetry.addData("Block Number", blockNumber);
            telemetry.update();
        }

        sleep(1000);

        if(blockNumber < 4 || blockNumber > 6)
        {
            blockNumber = 5;
        }

        if(blockNumber == 5)
        {
            encoderDrive(WHEEL_SPEED, 20, 8);

            servoSetup();

            straightenRight(1);

            while (opModeIsActive() && runtime.seconds() < 3)
            {
                skyStoneBot.getGripperAssembly().wheelIntake(INTAKE_SPEED);
                skyStoneBot.getChassisAssembly().moveForward(0.15);
            }

            skyStoneBot.getChassisAssembly().stopMoving();
            sleep(500);
            skyStoneBot.getGripperAssembly().wheelStop();

            skyStoneBot.getPlacementAssembly().slap();
            sleep(1000);

            skyStoneBot.getPlacementAssembly().grab(gripperGrab);
            sleep(1000);

            deliverStone();

            placeStone();

            skyStoneBot.getPlacementAssembly().swivel(swivelPlace);
            sleep(500);
            skyStoneBot.getPlacementAssembly().gripperSwivel(gripperSwivelPlace);


            //Second Block is blockNumber 2 so move back to 12 inches
            double distanceToDrive = DISTANCE_TO_BRIDGE - 16;
            telemetry.addData("To Drive:", distanceToDrive);
            telemetry.update();
            sleep(3000);

           encoderDrive(WHEEL_SPEED, 0.75 * distanceToDrive, 8);
           straightenRight(1);

           distanceToDrive = skyStoneBot.getNavigation().frontDistance() - 16;
           encoderDrive(WHEEL_SPEED, distanceToDrive, 4);

           encoderTurn(WHEEL_SPEED, -45, 5);

        }
        else if(blockNumber == 4)
        {
            encoderDrive(WHEEL_SPEED, 15, 8);

            servoSetup();

            straightenRight(1);

            encoderSide(WHEEL_SPEED, sideShift, 4);

            straightenRight(1);

            encoderDrive(WHEEL_SPEED, 5, 8);

            while (opModeIsActive() && runtime.seconds() < 3)
            {
                skyStoneBot.getGripperAssembly().wheelIntake(INTAKE_SPEED);
                skyStoneBot.getChassisAssembly().moveForward(0.15);
            }

            skyStoneBot.getChassisAssembly().stopMoving();
            sleep(500);
            skyStoneBot.getGripperAssembly().wheelStop();

            skyStoneBot.getPlacementAssembly().slap();
            sleep(1000);

            skyStoneBot.getPlacementAssembly().grab(gripperGrab);
            sleep(1000);

            deliverStone();


        }




        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
    }

    public void deliverStone()
    {
        encoderDrive(WHEEL_SPEED, -18, 5);

        straightenRight(1);

        double backDistance = skyStoneBot.getNavigation().backDistance();

        encoderDrive(WHEEL_SPEED, 24 - backDistance, 3);

        encoderTurn(WHEEL_SPEED, 90, 5);

        straightenRight(1);

        double distanceToDrive = DISTANCE_TO_BRIDGE - skyStoneBot.getNavigation().frontDistance();
        telemetry.addData("Distance to Drive", distanceToDrive);
        telemetry.update();
        sleep(2000);

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

    public void servoSetup()
    {
        //Servo Positions
        skyStoneBot.getChassisAssembly().openHook();
        skyStoneBot.getPlacementAssembly().swivel(swivelPlace);
        skyStoneBot.getPlacementAssembly().gripperSwivel(gripperSwivelPlace);
        skyStoneBot.getPlacementAssembly().grab(gripperPlace);
        skyStoneBot.getPlacementAssembly().slapperReturn();
    }

    public void straightenRight(double timeOut)
    {
        double angle = skyStoneBot.getNavigation().rightAngle();
        do
        {
         /*   ElapsedTime senseTime = new ElapsedTime();
            while (Math.abs(angle) > 15 && opModeIsActive() && senseTime.seconds() < timeOut) {
                angle = skyStoneBot.getNavigation().rightAngle();
                telemetry.addData("Angle", angle);
                telemetry.update();
            }
            */


            telemetry.addData("Angle", angle);
            telemetry.addData("Front Right", skyStoneBot.getNavigation().frontRightDistance());
            telemetry.addData("Back Right", skyStoneBot.getNavigation().backRightDistance());
            telemetry.update();

            if (Math.abs(angle) > 5 && Math.abs(angle) < 70) {
                encoderTurn(WHEEL_SPEED, angle, 3);
            }

            angle = skyStoneBot.getNavigation().rightAngle();
        }while(Math.abs(angle) > 10);
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

    }//end of encoderTurn
}
