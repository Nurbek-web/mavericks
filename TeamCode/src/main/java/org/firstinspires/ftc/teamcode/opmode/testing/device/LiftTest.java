package org.firstinspires.ftc.teamcode.opmode.testing.device;

import static java.lang.Thread.sleep;

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
    LiftSubsystem lift;

    public static double UP_LEFT = 0;
    public static double UP_RIGHT = 1;

    boolean openPixel = true;

    @Override
    public void init() {
        robot.init(hardwareMap);

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift = new LiftSubsystem();

        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {

//        if (gamepad1.right_bumper) {
//            double pos = robot.liftMotor.getCurrentPosition();
//            while (pos > -1000) {
//                pos = robot.liftMotor.getCurrentPosition();
//                robot.liftMotor.setPower(1);
//            }
//            robot.liftMotor.setPower(0);
//        }
//
//        if (gamepad1.left_bumper) {
//            lift.liftzero();
//        }
//
        robot.liftMotor.setPower(gamepad1.left_stick_y);

        telemetry.addData("liftPos", robot.liftMotor.getCurrentPosition());


//        telemetry.addData("upRight: ", robot.upRight.get);
//        telemetry.addData("downLeft: ", robot.downLeft.getPosition());
//        telemetry.addData("upBack: ", robot.upBack.getPosition());
//        telemetry.addData("upFront: ", robot.upFront.getPosition());
//        telemetry.addData("upBack: ", robot.upBack.getPosition());

        telemetry.update();
    }
}
