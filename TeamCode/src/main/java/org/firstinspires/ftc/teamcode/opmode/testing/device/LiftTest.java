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

@Config
@TeleOp(name = "LiftTest")
public class LiftTest extends OpMode {
    private PIDController controller;
    public DcMotorEx liftMotor;

    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;

    public static double  p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;
    private final double ticks_in_degree = 700/180.0;


    private final int lFirstLevelPos = 1400;
    private final int lSecondLevelPos = 700;
    private final int lDownPos = 0;

    @Override
    public void init() {
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
//        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        liftMotor.setTargetPosition(lDownPos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Turn the motor back on when we are done

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        controller = new PIDController(p, i ,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);
        int liftPos = liftMotor.getCurrentPosition();

        if (gamepad1.a) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
            telemetry.addLine("button B pressed");
            liftMotor.setTargetPosition(lFirstLevelPos);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(0.5);
        }

        if (gamepad1.b) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
            telemetry.addLine("button A pressed");
            liftMotor.setTargetPosition(lDownPos);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(0.5);
        }

        if (gamepad1.y) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
            telemetry.addLine("button Y pressed");
            liftMotor.setTargetPosition(-1000);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(0.5);
        }

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

        // Show the target position of the armMotor on telemetry
        telemetry.addData("Desired Position", desiredPosition);

        telemetry.update();
    }
}
