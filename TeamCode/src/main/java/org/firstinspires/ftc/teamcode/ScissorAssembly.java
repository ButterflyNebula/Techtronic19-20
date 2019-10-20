package org.firstinspires.ftc.teamcode;

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
}