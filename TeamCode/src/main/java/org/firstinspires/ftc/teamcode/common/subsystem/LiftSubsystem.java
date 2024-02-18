package org.firstinspires.ftc.teamcode.common.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class LiftSubsystem extends SubsystemBase {

    RobotHardware robot = RobotHardware.getInstance();
    private int liftLevel2=1400, liftLevel1=700;
    private static double curFirstServosPos = .5;
    private static double curSecondServosPos = .5;

    public LiftSubsystem(){}

    // lift the system
//    public void liftzero(){
////        robot.liftMotor.setTargetPosition(-robot.liftMotor.getCurrentPosition()); // 0
////        robot.liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
////        robot.liftMotor.setVelocity(200);
//          robot.liftMotor.setPower(0);
////          sleep(100);
//    }
//    public void liftFirstLevel(){
//        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.liftMotor.setTargetPosition(liftLevel1); // 700
//        robot.liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        robot.liftMotor.setVelocity(200);
//    }
//    public void liftSecondLevel(){
//        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.liftMotor.setTargetPosition(liftLevel2); // 1400
//        robot.liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        robot.liftMotor.setVelocity(200);
//    }

    public void extendOuttake() {
        // down
        this.robot.upRight.setPosition(0);
        this.robot.upLeft.setPosition(0.8);

    }

    public void intendOuttake() {
        // up
        this.robot.upRight.setPosition(0.8);
        this.robot.upLeft.setPosition(0.47);
    }

    // holding pixels
    public void closeOuttake()
    {
        this.robot.upFront.setPosition(0.535);
    }
    public void openOuttake(){
        this.robot.upFront.setPosition(0);
    }


    public void haltPixel(){
        this.robot.upBack.setPosition(.5);
    }
    public void unhaltPixel(){
        this.robot.upBack.setPosition(0);
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
