package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

import org.firstinspires.ftc.teamcode.common.util.wrappers.WEncoder;

@TeleOp(name = "HangTest")
@Config
public class Hang extends OpMode {
    public DcMotorEx hangLeftMotor;
    public DcMotorEx hangRightMotor;
    public Servo hangLeftServo;
    public Servo hangRightServo;
    public WEncoder extensionEncoder;

    public static double leftServoPos = 0.0;
    public static double rightServoPos = 0.0;


    @Override
    public void init() {
        this.hangLeftMotor = hardwareMap.get(DcMotorEx.class, "hangLeftMotor");
        hangLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.hangRightMotor = hardwareMap.get(DcMotorEx.class, "hangRightMotor");
        hangRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.hangLeftServo = hardwareMap.get(Servo.class, "hangLeftServo");
        this.hangRightServo = hardwareMap.get(Servo.class, "hangRightServo");

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            hangLeftServo.setPosition(0.6);
            hangRightServo.setPosition(0.3);
        }
        if (gamepad1.left_bumper) {
            hangLeftServo.setPosition(1);
            hangRightServo.setPosition(0.8);
        }

        // hang Closed Position
        // leftServoPos = 0.6 rightServoPos = 0.3

        // hang Opened Position
        // leftServoPos = 1 rightServoPos = 0.8

        telemetry.update();
    }

}
