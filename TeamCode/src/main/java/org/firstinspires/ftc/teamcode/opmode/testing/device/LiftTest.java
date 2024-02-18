package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;

@Config
@TeleOp(name = "LiftTest")
public class LiftTest extends OpMode {
    private final RobotHardware robot = RobotHardware.getInstance();



    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;

    public static int TARGET = 0;
    public static double POWER = 0;
    private final int lFirstLevelPos = 1400;
    private final int lSecondLevelPos = 700;
    private final int lDownPos = 0;
    private static double upRightPos = 0;
    private static double upBackPos = 0;
    private static double upFrontPos = 0;
    private static double downLeftPos = 0;

    LiftSubsystem lift;


    public static double LOW_POS = 0;
    public static double HIGH_POS = 1;

    public static double upRightPower = 0.8;
    public static double upLeftPower = 0.47;
    public static double upLeftPowerZero = 1;
    public static double upRightPowerZero = 0;
    public static double closeOuttakePos = 0.535;
    public static double openOuttakePos = 0;

    boolean openPixel = true;

    @Override
    public void init() {
        robot.init(hardwareMap);

        robot.upRight.setDirection(Servo.Direction.REVERSE);
        robot.upLeft.setDirection(Servo.Direction.FORWARD);

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift = new LiftSubsystem();

    }

    @Override
    public void loop() {
        robot.liftMotor.setPower(POWER);

        if (gamepad1.right_bumper) { // upFront = 0; (opened)
            robot.upLeft.setPosition(HIGH_POS); // upFront = 0.535 (closed)
        }
        if (gamepad1.left_bumper) {
            robot.upRight.setPosition(LOW_POS);
        }

//        if (gamepad2.right_bumper) { // down
//            robot.upRight.setPosition(0.8);
//            robot.upLeft.setPosition(1);
//        }
//
//        if (gamepad2.left_bumper) { // up
//            robot.upRight.setPosition(0);
//            robot.upLeft.setPosition(0.47);
//        }

        // bullsiht fuuuuuu ((((
//        if(gamepad2.x){ // extendOuttake
//            robot.upRight.setPosition(upRightPower);
//            robot.upLeft.setPosition(upLeftPower);
//        }
//        if(gamepad2.y){ // reverseExtend
//            robot.upRight.setPosition(upRightPowerZero);
//            robot.upLeft.setPosition(upLeftPowerZero);
//        }
//        if(gamepad2.a){ // closeOuttake open
//            if(openPixel){ // close
//                robot.upFront.setPosition(closeOuttakePos);
//            }else{ // open
//                robot.upFront.setPosition(openOuttakePos);
//            }
//        }

//        if (gamepad1.a){
////        robot.liftMotor.setTargetPosition(-robot.liftMotor.getCurrentPosition()); // 0
////        robot.liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
////        robot.liftMotor.setVelocity(200);
//            robot.liftMotor.setPower(0);
////          sleep(100);
//        }
//        if (gamepad1.b) {
//            robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.liftMotor.setTargetPosition(liftLevel1); // 700
//            robot.liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            robot.liftMotor.setVelocity(200);
//        }
//        if (gamepad1.x){
//            robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.liftMotor.setTargetPosition(liftLevel2); // 1400
//            robot.liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//            robot.liftMotor.setVelocity(200);
//        }


        telemetry.addData("power", POWER);


//        telemetry.addData("upRight: ", robot.upRight.get);
//        telemetry.addData("downLeft: ", robot.downLeft.getPosition());
//        telemetry.addData("upBack: ", robot.upBack.getPosition());
//        telemetry.addData("upFront: ", robot.upFront.getPosition());
//        telemetry.addData("upBack: ", robot.upBack.getPosition());

        telemetry.update();
    }
}
