package org.firstinspires.ftc.teamcode.qualifier2;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Frame;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name = "BitmapTest")
public class GetBitmap extends LinearOpMode {
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;


    private static final String VUFORIA_KEY =
            "AVsSQB3/////AAABGehU0OjxREoznvNKEBqbvmskci8syRYfMKE0XlaGnZpw68DAZV19s7dfqc0vWrY78bAO2Ym2n1T2rDvNBOVVbMWxXIRo2c18JH6/c2fcKT1bRKxsG7bYq69+n9IHmKedY6rmTU1VOZZdtSTXh7exMsl67IAcnCZ0/ec+P+ZMpkK5v4X8d27rbEigGqqHayGe1/lG2afzgcHY7QxjJ/x5O4yGmVVs8wdzdupke19U+M8Z/x0FcYIfTAHuXcaydEL+h/w/ppcuNarD2ggo2BxdWeOGLx5GOin1yruVfvDAazPEuI0m3yEwXQNZ4e0ar2G0jDCZpAJPJcJRRVttBMwPoAvzTwySUx3qI1eNSJRAH+bk";


    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

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
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    SkyBot skyStoneBot = new SkyBot();

    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "It initialized1");
        telemetry.update();

        skyStoneBot.initRobot(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        telemetry.addData("Status", "It initialized2");
        telemetry.update();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        telemetry.addData("Status", "It initialized3");
        telemetry.update();

        parameters.cameraName = skyStoneBot.getNavigation().robotHardware.webcam;

        telemetry.addData("Status", "It initialized4");
        telemetry.update();

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        telemetry.addData("Status", "It initialized5");
        telemetry.update();

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        telemetry.addData("Status", "It initialized6");
        telemetry.update();

        vuforia.setFrameQueueCapacity(2);

        telemetry.addData("Status", "It initialized7");
        telemetry.update();

        Frame frame = vuforia.getFrameQueue().take();

        telemetry.addData("Status", "It initialized8");
        telemetry.update();

        Bitmap bitmap = vuforia.convertFrameToBitmap(frame);

        telemetry.addData("Status", "It initialized9");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            for(int i = 0; i<640; i+=12){
                int currentPixel = bitmap.getPixel(i, 360);
                telemetry.addData("Red",Color.red(currentPixel));
                telemetry.addData("Green",Color.green(currentPixel));
                telemetry.addData("Blue",Color.blue(currentPixel));
                telemetry.addData("X cord",i);
                telemetry.update();
                sleep(2000);
            }
            telemetry.addData("Height",bitmap.getHeight());
            telemetry.addData("Width", bitmap.getWidth());
            telemetry.update();
        }
    }
}