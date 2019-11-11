package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class ChassisAssembly
{

    RobotHardware robotHardware;

    protected ChassisAssembly(RobotHardware hardware)
    {
        robotHardware = hardware;
    }

    private boolean rightDown = true;
    private boolean leftDown = true;

    /**
     *-----------------------------------------------------------------------------------
     * WHEEL CONTROLS
     * This is used to move the robot around.  The robot can move in all directions with
     * the use of Mecanum wheels
     * ----------------------------------------------------------------------------------
     */

    /**
     * Moves the robot forward by giving power to all four wheels
     * @param speed at which the wheel motors will turn
     */
    protected void moveForward(double  speed)
    {
        robotHardware.frontLeftWheel.setPower(speed);
        robotHardware.frontRightWheel.setPower(speed);
        robotHardware.backLeftWheel.setPower(speed);
        robotHardware.backRightWheel.setPower(speed);
    }

    /**
     * Moves the robot backwards by giving negative power to all four wheels
     * @param speed at which the wheel motors will turn
     */
    protected void moveBackwards(double speed)
    {
        robotHardware.frontLeftWheel.setPower(-speed);
        robotHardware.frontRightWheel.setPower(-speed);
        robotHardware.backLeftWheel.setPower(-speed);
        robotHardware.backRightWheel.setPower(-speed);
    }

    /**
     * Rotates the robot to the right by giving the left wheels power and the right
     * wheels negative power
     * @param speed at which the wheel motors will turn
     */
    protected void turnRight (double speed)
    {
        robotHardware.frontLeftWheel.setPower(speed);
        robotHardware.backLeftWheel.setPower(speed);
        robotHardware.frontRightWheel.setPower(-speed);
        robotHardware.backRightWheel.setPower(-speed);
    }

    /**
     * Rotates the robot to the left by giving the right wheels power and the left
     * wheels negative power
     * @param speed at which the wheel motors will turn
     */
    protected void turnLeft (double speed)
    {
        robotHardware.frontLeftWheel.setPower(-speed);
        robotHardware.backLeftWheel.setPower(-speed);
        robotHardware.frontRightWheel.setPower(speed);
        robotHardware.backRightWheel.setPower(speed);
    }

    /**
     * Moves the robot to the right laterally.  The front left wheel and back right wheel
     * are given negative power and the front right wheel and back left wheel are given power.
     * @param speed at which the wheel motors will turn
     */
    protected void moveLeft (double speed)
    {
        robotHardware.frontLeftWheel.setPower(speed);
        robotHardware.backLeftWheel.setPower(-speed);
        robotHardware.frontRightWheel.setPower(-speed);
        robotHardware.backRightWheel.setPower(speed);
    }

    /**
     * Moves the robot to the left laterally.  The front right wheel and back left wheel
     * are given negative power and the front left wheel and back right wheel are given power.
     * @param speed at which the wheel motors will turn
     */
    protected void moveRight (double speed)
    {
        robotHardware.frontLeftWheel.setPower(-speed);
        robotHardware.backLeftWheel.setPower(speed);
        robotHardware.frontRightWheel.setPower(speed);
        robotHardware.backRightWheel.setPower(-speed);
    }

    /**
     * Stops the robot by setting the power of all wheels to 0
     */
    protected void stopMoving()
    {
        robotHardware.frontLeftWheel.setPower(0);
        robotHardware.frontRightWheel.setPower(0);
        robotHardware.backLeftWheel.setPower(0);
        robotHardware.backRightWheel.setPower(0);
    }



    /**
     *-----------------------------------------------------------------------------------
     * ENCODER WHEEL CONTROLS
     * This is used during the autonomous mode to ensure that the robot moves to accurate
     * distances.
     * There are many different methods that are used when running with encoder.
     * ----------------------------------------------------------------------------------
     */

    /**
     * This method changes the mode the wheels are running in to passed run mode.
     * Some such run modes include RUN_USING_ENCODER and RUN_TO_POSITION, etc.
     * @param mode which the wheels should be set to running in
     */
    public void setMode(DcMotor.RunMode mode)
    {
        robotHardware.backLeftWheel.setMode(mode);
        robotHardware.backRightWheel.setMode(mode);
        robotHardware.frontLeftWheel.setMode(mode);
        robotHardware.frontRightWheel.setMode(mode);
    }

    /**
     * The follow four methods are used to individually set the power for each wheel
     * This is needed when using the encoder
     * @param speed the power that the motor will be set to
     */
    public void setBackLeftWheelPower(double speed) {robotHardware.backLeftWheel.setPower(speed);}
    public void setBackRightWheelPower(double speed) {robotHardware.backRightWheel.setPower(speed);}
    public void setFrontLeftWheelPower(double speed) {robotHardware.frontLeftWheel.setPower(speed);}
    public void setFrontRightWheelPower(double speed) { robotHardware.frontRightWheel.setPower(speed); }


    /**
     * The following four methods test each wheel if they are busy in that the wheel's
     * motor is powered and active
     * @return whether or not the wheel is busy where true means it is busy and false
     * means it is not busy
     */
    public boolean isBackLeftWheelBusy()
    {
        return robotHardware.backLeftWheel.isBusy();
    }
    public boolean isBackRightWheelBusy()
    {
        return robotHardware.backRightWheel.isBusy();
    }
    public boolean isFrontLeftWheelBusy()
    {
        return robotHardware.frontLeftWheel.isBusy();
    }
    public boolean isFrontRightWheelBusy()
    {
        return robotHardware.frontRightWheel.isBusy();
    }


    /**
     * Resets the encoders of all the wheels' motors and changes them to encoder mode
     */
    public void changeToEncoderMode()
    {
        robotHardware.backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware.backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware.frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.frontRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robotHardware.frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotHardware.frontLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * The following four methods gets the current position of the wheels' motors
     * @return an integer value representing the current position of the wheel's motor
     */
    public int getBackRightWheelCurrentPosition() {return robotHardware.backRightWheel.getCurrentPosition();}
    public int getBackLeftWheelCurrentPosition() {return robotHardware.backLeftWheel.getCurrentPosition();}
    public int getFrontRightWheelCurrentPosition() {return robotHardware.frontRightWheel.getCurrentPosition();}
    public int getFrontLeftWheelCurrentPosition() {return robotHardware.frontLeftWheel.getCurrentPosition();}

    //Hook Positions
    public double getRhookPosition() {return robotHardware.rightHook.getPosition();}
    public double getLhookPosition() {return robotHardware.leftHook.getPosition();}

    /**
     * The following four methods sets the target position to which each wheel's motor should
     * run to.
     */
    public void setBackLeftWheelTargetPosition(int position) {robotHardware.backLeftWheel.setTargetPosition(position);}
    public void setBackRightWheelTargetPosition(int position) {robotHardware.backRightWheel.setTargetPosition(position);}
    public void setFrontLeftWheelTargetPosition(int position) {robotHardware.frontLeftWheel.setTargetPosition(position);}
    public void setFrontRightWeelTargetPosition(int position) {robotHardware.frontRightWheel.setTargetPosition(position);}


    //Hook Controls
    public void openHook(){
        robotHardware.rightHook.setPosition(0.2);
        robotHardware.leftHook.setPosition(0.9);
        rightDown = false;
        leftDown = false;
    }
    public void closeHook() {
        robotHardware.rightHook.setPosition(0.9);
        robotHardware.leftHook.setPosition(0);
        rightDown = true;
        leftDown = true;
    }

    public void moveRightHook()
    {
        if(rightDown)
        {
            robotHardware.rightHook.setPosition(0.2);
            rightDown = false;
        }
        else
        {
            robotHardware.rightHook.setPosition(0.9);
            rightDown = true;
        }
    }

    public void moveLeftHook()
    {
        if(leftDown)
        {
            robotHardware.leftHook.setPosition(0.9);
            leftDown = false;
        }
        else
        {
            robotHardware.leftHook.setPosition(0);
            leftDown = true;
        }
    }
}
