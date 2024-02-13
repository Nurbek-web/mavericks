//package org.firstinspires.ftc.teamcode.opmode.testing.device;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.hardware.motors.MotorEx;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
//import org.firstinspires.ftc.teamcode.common.util.wrappers.WEncoder;
//
//@TeleOp(name = "arm")
//@Disabled
//public class Arm extends OpMode {
//    public AbsoluteAnalogEncoder extensionPitchEncoder;
//    public AnalogInput extensionPitchEnc;
//
//    public WEncoder extensionEncoder;
//
//    public DcMotor liftMotor;
//
//    @Override
//    public void init() {
//
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
//
//        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
//
//    }
//
//    @Override
//    public void loop() {
//        liftMotor.setPower(1);
//
//        telemetry.addData("stick_y", gamepad1.left_stick_y);
//        telemetry.update();
//    }
//
//}
