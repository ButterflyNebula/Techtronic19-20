package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ScissorAssembly {

    RobotHardware robotHardware;

    protected ScissorAssembly(RobotHardware hardware) {
        robotHardware = hardware;
    }


    /**
     * Moves the scissor lift upw and down
     *
     * @param speed at which the scissor motors will turn. A negative value will
     *              move the scissor lift down
     */
    protected void move(double speed) {
        robotHardware.leftScissor.setPower(speed);
        robotHardware.rightScissor.setPower(speed);
    }

    protected void moveRight(double speed)
    {
        robotHardware.rightScissor.setPower(-speed);
    }

    protected void moveLeft(double speed)
    {
        robotHardware.leftScissor.setPower(-speed);
    }


    //Encoder Methods
    protected int getRightPosition() { return robotHardware.rightScissor.getCurrentPosition();}
    protected int getLeftPosition() { return robotHardware.leftScissor.getCurrentPosition();}

    protected void setMode(DcMotor.RunMode mode){
        robotHardware.rightScissor.setMode(mode);
        robotHardware.leftScissor.setMode(mode);
    };

    protected boolean isLeftBusy(){return robotHardware.leftScissor.isBusy();}
    protected boolean isRightBusy(){return robotHardware.rightScissor.isBusy();}

    protected void setRightPosition(int position){robotHardware.rightScissor.setTargetPosition(position);};
    protected void setLeftPosition(int position){robotHardware.leftScissor.setTargetPosition(position);};

    protected boolean leftTouch(){ return robotHardware.leftScissorTouch.getState();}
    protected boolean rightTouch(){ return robotHardware.rightScissorTouch.getState();}


}