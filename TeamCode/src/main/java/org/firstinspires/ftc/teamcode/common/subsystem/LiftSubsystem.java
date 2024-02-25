package org.firstinspires.ftc.teamcode.common.subsystem;

import static java.lang.Thread.sleep;

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
    public void liftzero(){

        intendOuttake();

        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setTargetPosition(-1200);
        robot.liftMotor.setPower(0.5);
    }
    public void liftFirstLevel() throws InterruptedException {
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double pos = robot.liftMotor.getCurrentPosition();
        while (pos > -1000) {
            pos = robot.liftMotor.getCurrentPosition();
            robot.liftMotor.setPower(1);
        }
        robot.liftMotor.setPower(0);

        extend1Outtake();
        openOuttake();
    }
//    public void liftSecondLevel(){
//        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.liftMotor.setTargetPosition(liftLevel2); // 1400
//        robot.liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        robot.liftMotor.setVelocity(200);
//    }

    public void extend2Outtake() { // second level
        this.robot.upRight.setPosition(.15);
        this.robot.downLeft.setPosition(.15);
        this.robot.upLeft.setPosition(1);

    }

    public void extend1Outtake() { // first level
        this.robot.upRight.setPosition(.4); // 0.2
        this.robot.downLeft.setPosition(.4);
        this.robot.upLeft.setPosition(.8);

    }
    public void intendOuttake() { // init state of lift
        this.robot.upRight.setPosition(.99);
        this.robot.downLeft.setPosition(.99);
        this.robot.upLeft.setPosition(.47);
    }

    // holding pixels
    public void closeOuttake()
    {
        this.robot.upFront.setPosition(0);
    }
    public void openOuttake(){
        this.robot.upFront.setPosition(0.535);
    }


    public void haltPixel(){
        this.robot.upBack.setPosition(.5);
    }
    public void unhaltPixel(){
        this.robot.upBack.setPosition(0);
    }
}
