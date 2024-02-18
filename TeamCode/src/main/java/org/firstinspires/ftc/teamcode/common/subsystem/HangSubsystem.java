package org.firstinspires.ftc.teamcode.common.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
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

    public void openServo() {
        robot.hangLeftServo.setPosition(1);
        robot.hangRightServo.setPosition(0.8);
        Globals.HangServoOpened = Globals.HangServoState.OPENED;
    }

    public void closeServo() {
        robot.hangLeftServo.setPosition(0.6);
        robot.hangRightServo.setPosition(0.3);
    }

    public void reverseMotors() {
        Globals.HangServoOpened = Globals.HangServoState.REVERSE;
    }
}
