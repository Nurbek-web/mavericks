//package org.firstinspires.ftc.teamcode.opmode.teleop;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.ConditionalCommand;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.arcrobotics.ftclib.hardware.motors.MotorEx;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.common.hardware.Globals;
//import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.common.subsystem.HangSubsystem;
//import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;
//import org.firstinspires.ftc.teamcode.common.util.wrappers.WEncoder;
//
//@TeleOp(name = "TestTeleop")
//public class TestTeleop extends OpMode {
//    public AnalogInput extensionPitchEnc;
//
//    public WEncoder extensionEncoder;
//
//    private final RobotHardware robot = RobotHardware.getInstance();
//    private GamepadEx gamepadEx;
//    private GamepadEx gamepadEx2;
//
//    CommandScheduler scheduler = CommandScheduler.getInstance();
//
//    private double loopTime = 0.0;
//
//    IntakeSubsystem intake;
//    HangSubsystem hang;
//    LiftSubsystem lift;
//
//    public int liftLevel2 = 1400;
//    public int liftLevel1 = 700;
//
//    public DcMotor hangLeftMotor;
//    public DcMotor hangRightMotor;
//
//    @Override
//    public void init() {
//
//        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
////
//        hangLeftMotor = hardwareMap.get(DcMotor.class, "hangLeftMotor");
//        hangRightMotor = hardwareMap.get(DcMotor.class, "hangRightMotor");
//
//        scheduler.reset();
//
//        intake = new IntakeSubsystem();
//        hang = new HangSubsystem();
//        lift = new LiftSubsystem();
//
//        Globals.IS_AUTO = false;
//        Globals.raiseIntake();
//
//        gamepadEx = new GamepadEx(gamepad1);
//        gamepadEx2 = new GamepadEx(gamepad2);
//
//        robot.init(hardwareMap);
//
//        robot.read();
//
////        gamepadEx.getGamepadButton(GamepadKeys.Button.Y )
////                   .whenPressed(
////                           new InstantCommand(() -> {
////                               intake.raiseServo();
////                               telemetry.addLine("RaiseServo");
////                           })));
////                        new ConditionalCommand(
////                            new InstantCommand(() -> intake.lowerServo()),
////                            new InstantCommand(() -> intake.raiseServo()),
////                            () -> intake.getServoState()==IntakeSubsystem.ServoState.RAISED
////                ));
//
//
////        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)
////                .whenPressed(
////
////                        new InstantCommand(() -> {
////                            telemetry.addLine("lowerServo");
////                            telemetry.update();
////                            intake.lowerServo();
////
////                        }));
////
////        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
////                .whenPressed(
////                        new InstantCommand(() -> {
////                            telemetry.addLine("RaiseServo");
////                            telemetry.update();
////                            intake.raiseServo();
////                        }));
//
//    }
//
//    @Override
//    public void loop() {
//        scheduler.run();
//        robot.clearBulkCache();
//        robot.read();
//        robot.periodic();
//        robot.write();
//
////        if(gamepadEx.wasJustPressed(GamepadKeys.Button.X)){
////            intake.raiseServo();
////            telemetry.addLine("RaiseServo");
////        }
////
//        hangLeftMotor.setPower(gamepad1.left_stick_y);
//        hangRightMotor.setPower(gamepad1.right_stick_y);
//
//
//        telemetry.update();
//
//    }
//
//}
