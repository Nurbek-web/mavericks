//package org.firstinspires.ftc.teamcode.opmode.teleop;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.ConditionalCommand;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.CRServoImplEx;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
////import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
////import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ArmCommand;
////import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.PivotStateCommand;
////import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.ClawDepositCommand;
////import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.DepositExtendCommand;
////import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.DepositRetractionCommand;
////import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.HeightChangeCommand;
////import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.IntakeExtendCommand;
////import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.IntakeRetractCommand;
////import org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand.HeightChangeCommand;
//import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
//import org.firstinspires.ftc.teamcode.common.hardware.Globals;
//import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.common.subsystem.DroneSubsystem;
////import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.common.subsystem.HangSubsystem;
//import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;
//import org.firstinspires.ftc.teamcode.common.util.InverseKinematics;
//import org.firstinspires.ftc.teamcode.common.util.MathUtils;
//
//@Config
//@TeleOp(name = "SampleTeleop")
//public class SampleTeleop extends CommandOpMode {
//
//    private final RobotHardware robot = RobotHardware.getInstance();
//    private GamepadEx gamepadEx;
//    private GamepadEx gamepadEx2;
//
//    CommandScheduler scheduler = CommandScheduler.getInstance();
//
//    private double loopTime = 0.0;
////    private boolean lastJoystickUp = false;
////    private boolean lastJoystickDown = false;
////    private boolean extendIntake = true;
//
//    IntakeSubsystem intake;
//    HangSubsystem hang;
//    LiftSubsystem lift;
//
//    public int liftLevel2 = 1400;
//    public int liftLevel1 = 700;
//
//    @Override
//    public void initialize() {
//        scheduler.reset();
//
//        intake = new IntakeSubsystem();
//        hang = new HangSubsystem();
//        lift = new LiftSubsystem();
//
//        Globals.IS_AUTO = false;
//        Globals.lowerIntake();
//
//        gamepadEx = new GamepadEx(gamepad1);
//        gamepadEx2 = new GamepadEx(gamepad2);
//
//        robot.init(hardwareMap);
//
//        // binding commands
//
//
//        // initial hang servo positions
//        // right 0.5 - 1
//        // left 1 - 0.5
////        robot.hangRight.setPosition(0.5);
////        robot.hangLeft.setPosition(1);
//
//        robot.read();
//        while (opModeInInit()) {
//            telemetry.addLine("Robot Initialized.");
//            telemetry.update();
//        }
//    }
//
//    @Override
//    public void run() {
//        scheduler.run();
//        robot.clearBulkCache();
//        robot.read();
//        robot.periodic();
//        robot.write();
//
//////         G1 - Drivetrain Control
////        robot.drivetrain.set(new Pose(gamepad1.left_stick_x, -gamepad1.left_stick_y, MathUtils.joystickScalar(-gamepad1.left_trigger + gamepad1.right_trigger, 0.01)), 0);
//        robot.drivetrain.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, gamepad1.right_stick_x);
//
////        boolean currentJoystickUp = gamepad1.right_stick_y < -0.5 || gamepad2.right_stick_y < -0.5;
////        boolean currentJoystickDown = gamepad1.right_stick_y > 0.5 || gamepad2.right_stick_y > 0.5;
//
//        // FIRST DRIVER
//
//        // lift
//        int liftCurPosition = robot.liftMotor.getCurrentPosition();
//        if(gamepadEx2.wasJustPressed(GamepadKeys.Button.X) && liftCurPosition!=0){
//            scheduler.schedule(new InstantCommand(() -> {
//                lift.liftSecondLevel();
//            }));
//        }else if(gamepadEx2.wasJustPressed(GamepadKeys.Button.Y) && liftCurPosition!=0){
//            scheduler.schedule(new InstantCommand(() -> {
//                lift.liftFirstLevel();
//            }));
//        }else if(gamepadEx2.wasJustPressed(GamepadKeys.Button.A)){
//            scheduler.schedule(new InstantCommand(() -> {
//                lift.liftzero();
//            }));
//        }
//
//
//        // hang
//        // right 0.5 - 1
//        // left 1 - 0.5
//        if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
//            scheduler.schedule(new InstantCommand(() -> {
//                try {
//                    hang.hang();
//                }catch(InterruptedException err){
//                    telemetry.addLine("Hang Interrupted");
//                }
//                    })
////                    new ConditionalCommand(
////                    new InstantCommand(() -> {
////                        hang.hang();
////                    }),
////                    new InstantCommand(() -> {
////                        hang.unhang();
////                    }),
////                    () -> hang.curhangState == HangSubsystem.HangState.DISABLED
////            )
//            );
//        }
//
//        // intake
//        if (gamepadEx.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
//            scheduler.schedule(new ConditionalCommand(
//                    new InstantCommand(() -> intake.lowerServo()),
//                    new InstantCommand(() -> intake.raiseServo()),
//                    () -> !Globals.INTAKE_LOWERED
//            ));
//        }
//
//        if(gamepadEx.wasJustPressed(GamepadKeys.Button.A)){
//            scheduler.schedule(new InstantCommand(() -> intake.runIntake()));
//        }else if(gamepadEx.wasJustPressed(GamepadKeys.Button.B)) {
//            scheduler.schedule(new InstantCommand(() -> intake.stopIntake()));
//        }
//
//        if(gamepadEx.wasJustPressed(GamepadKeys.Button.Y)){
//            scheduler.schedule(new InstantCommand(() -> intake.releaseExtra()));
//        }else if(gamepadEx.wasJustReleased(GamepadKeys.Button.Y)){
//            scheduler.schedule(new InstantCommand(() -> intake.stopIntake()));
//        }
//
//        // SECOND DRIVER
//        if (gamepadEx2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
//            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//
//        double loop = System.nanoTime();
//        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//        loopTime = loop;
//        telemetry.update();
//    }
//}
