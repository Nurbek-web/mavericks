package org.firstinspires.ftc.teamcode.common.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class    LiftSubsystem extends SubsystemBase {

    RobotHardware robot = RobotHardware.getInstance();
    private int liftLevel2=1400, liftLevel1=700;

    public LiftSubsystem(){}

    public void liftzero(){
        robot.liftMotor.setTargetPosition(-robot.liftMotor.getCurrentPosition()); // 0
        robot.liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setVelocity(200);
    }
    public void liftFirstLevel(){
        robot.liftMotor.setTargetPosition(liftLevel1); // 700
        robot.liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setVelocity(200);
    }
    public void liftSecondLevel(){
        robot.liftMotor.setTargetPosition(liftLevel2); // 1400
        robot.liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setVelocity(200);
    }
}
