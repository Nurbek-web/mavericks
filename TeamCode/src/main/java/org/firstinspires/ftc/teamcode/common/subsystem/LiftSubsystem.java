package org.firstinspires.ftc.teamcode.common.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class    LiftSubsystem extends SubsystemBase {

    RobotHardware robot = RobotHardware.getInstance();
    private int liftLevel2=1400, liftLevel1=700;
    private static double curFirstServosPos = .5;
    private static double curSecondServosPos = .5;

    public LiftSubsystem(){}

    // lift the system
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

    // holding pixels
    public void closeOuttake(){
        robot.upFront.setPosition(1); // or what should it be
    }
    public void openOuttake(){
        robot.upFront.setPosition(.5);
    }

    public void haltPixel(){
        robot.upBack.setPosition(.5);
    }
    public void unhaltPixel(){
        robot.upBack.setPosition(0);
    }

    // control lift position
    public void moveFirstServos(double amount){ // amount can be 0.1; 0.2; and so on
        if(Globals.liftLevel==Globals.LiftLevel.FIRST){
            return;
        }
        if(amount==0.0){
            robot.upRight.setPosition(.5);
            robot.upLeft.setPosition(.5);
            return;
        }
        if(curFirstServosPos+amount>=1.0 || curFirstServosPos+amount<0){
            return;
        }
        robot.upRight.setPosition(curFirstServosPos+amount);
        robot.upLeft.setPosition(curFirstServosPos+amount);

    }

    public void moveSecondServos(double amount){ // what were the servo names???
        if(Globals.liftLevel==Globals.LiftLevel.FIRST){
            return;
        }
        if(amount==0.0){
            robot.upRight.setPosition(.5);
            robot.upLeft.setPosition(.5);
            return;
        }
        if(curSecondServosPos+amount>=1.0 || curSecondServosPos+amount<0){
            return;
        }
        robot.upRight.setPosition(curSecondServosPos+amount);
        robot.upLeft.setPosition(curSecondServosPos+amount);
    }
    public static double getCurFirstServosPos(){ return curFirstServosPos; }
    public static double getCurSecondServosPos(){ return curSecondServosPos; }
}
