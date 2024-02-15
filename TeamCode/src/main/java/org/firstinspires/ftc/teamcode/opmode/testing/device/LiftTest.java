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

@Config
@TeleOp(name = "LiftTest")
public class LiftTest extends OpMode {
    private PIDController controller;
    public DcMotorEx liftMotor;

    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;

    public static int target = 0;
    private final double ticks_in_degree = 700/180.0;


    private final int lFirstLevelPos = 1400;
    private final int lSecondLevelPos = 700;
    private final int lDownPos = 0;
    private static double power = 0.5;

    private static double upRightPos = 0;
    private static double upBackPos = 0;
    private static double upFrontPos = 0;
    private static double downLeftPos = 0;


    public Servo upRight;
    public Servo upBack;
    public Servo upFront;
    public Servo downLeft;
    @Override
    public void init() {
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
//        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        liftMotor.setTargetPosition(lDownPos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Turn the motor back on when we are done


        this.upRight = hardwareMap.get(Servo.class, "upRight");
        this.upBack = hardwareMap.get(Servo.class, "upBack");
        this.upFront = hardwareMap.get(Servo.class, "upFront");
        this.downLeft = hardwareMap.get(Servo.class, "downLeft");

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop() {
        int liftPos = liftMotor.getCurrentPosition();

//        if (gamepad1.a) {
//            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
//            telemetry.addLine("button B pressed");
//            liftMotor.setTargetPosition(lFirstLevelPos);
//            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            liftMotor.setPower(0.5);
//        }
//
//        if (gamepad1.b) {
//            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
//            telemetry.addLine("button A pressed");
//            liftMotor.setTargetPosition(lDownPos);
//            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            liftMotor.setPower(0.5);
//        }
//
//        if (gamepad1.y) {
//            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
//            telemetry.addLine("button Y pressed");
//            liftMotor.setTargetPosition(-1000);
//            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            liftMotor.setPower(0.5);
//        }

        upRight.setPosition(upRightPos);
        upBack.setPosition(upBackPos);
        upFront.setPosition(upFrontPos);
        downLeft.setPosition(downLeftPos);


        liftMotor.setTargetPosition(target);
        liftMotor.setPower(power);

//        if (gamepadEx.wasJustPressed(GamepadKeys.Button.Y)) {
//            liftMotor.setTargetPosition(lDownPos);
//            liftMotor.setPower(0.5);
//        }

        // Get the current position of the armMotor
        double position = liftMotor.getCurrentPosition();

        // Get the target position of the armMotor
        double desiredPosition = liftMotor.getTargetPosition();

        // Show the position of the armMotor on telemetry
        telemetry.addData("Encoder Position", position);

        telemetry.addData("power", power);

        // Show the target position of the armMotor on telemetry
        telemetry.addData("Desired Position", desiredPosition);

        telemetry.update();
    }
}
