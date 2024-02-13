package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.FtcDashboard;
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

@TeleOp(name = "hang")
@Disabled
public class Hang extends OpMode {
    public DcMotorEx hangLeftMotor;
    public DcMotorEx hangRightMotor;
    public Servo hangLeftServo;
    public Servo hangRightServo;
    public WEncoder extensionEncoder;

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

        telemetry.addData("position 2", extensionEncoder.getPosition());
        telemetry.update();
    }

}
