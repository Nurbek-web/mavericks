package org.firstinspires.ftc.teamcode.common.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;

public class HangSubsystem extends SubsystemBase {
    private final RobotHardware robot = RobotHardware.getInstance();

    public HangState curhangState = HangState.DISABLED;

    public enum HangState {
        ACTIVE,
        DISABLED
    }

    public HangSubsystem() {}

    public void hang() throws InterruptedException {
        this.robot.hangLeftMotor.setPower(0.8);
        this.robot.hangRightMotor.setPower(0.8);
    }

    public void openHang() {
        this.robot.hangLeftServo.setPosition(0.5);
        this.robot.hangRightServo.setPosition(0.5);
        curhangState = HangState.ACTIVE;
    }


    public void unhang(){
        this.robot.hangLeftServo.setPosition(.5);
        this.robot.hangRightServo.setPosition(1);
        curhangState = HangState.DISABLED;
    }

}
