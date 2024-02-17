package org.firstinspires.ftc.teamcode.opmode.testing.device;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Config
@TeleOp(name = "IntakeTest")
public class Intake extends OpMode {
    private final RobotHardware robot = RobotHardware.getInstance();

    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;

    public static double lowPos = 0.1;
    public static double highPos = 0.9;
    @Override
    public void init() {
        robot.init(hardwareMap);
        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop() {
        robot.drivetrain.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, gamepad1.right_stick_x);

        if (gamepad1.a) {
            robot.intakeMotor.setPower(1);
        }

        if (gamepad1.b) {
            robot.intakeMotor.setPower(0);
        }

        if (gamepad1.x) {
            robot.intakeMotor.setPower(-1);
        }

        if (gamepad1.left_bumper) {
            robot.intakeServo.setPosition(lowPos);
        }

        if (gamepad1.right_bumper) {
            robot.intakeServo.setPosition(highPos);
        }




        telemetry.update();
    }
}
