package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.DroneSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.HangSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.util.InverseKinematics;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;
import org.firstinspires.ftc.teamcode.common.util.logging.CSVInterface;
import org.firstinspires.ftc.teamcode.common.util.logging.LogType;
import org.firstinspires.ftc.teamcode.common.util.logging.Logger;

@Config
@TeleOp(name = "Solo")
public class Solo extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;

    private double loopTime = 0.0;
    private boolean lastJoystickUp = false;
    private boolean lastJoystickDown = false;

    private boolean extendIntake = true;


    IntakeSubsystem intake;
    HangSubsystem hang;
    LiftSubsystem lift;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Globals.IS_AUTO = false;
        Globals.lowerIntake();

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot.init(hardwareMap);

        intake = new IntakeSubsystem();

        // G1 - Intake Control
        gamepadEx.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ConditionalCommand(
                        new InstantCommand(() -> {
                            intake.raiseServo();
                            telemetry.addLine("raiseServo");
                        }),
                        new InstantCommand(() -> {
                            intake.lowerServo();
                            telemetry.addLine("loweredServo");
                        }),
                        () -> Globals.INTAKE_LOWERED
                ));

        // G1 - Intake Roll Control
        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(
                        new InstantCommand(() -> {
                            intake.runIntake();
                            telemetry.addLine("runIntake");
                        })
                );

        gamepadEx.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(
                        new InstantCommand(() -> {
                            intake.stopIntake();
                            telemetry.addLine("stopIntake");
                        })
                );
        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(
                        new InstantCommand(() -> {
                            intake.releaseExtra();
                            telemetry.addLine("releaseExtra");
                        })
                );

        gamepadEx2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(
                        new InstantCommand(() -> {
                            hang.openHang();
                            telemetry.addLine("hangopen");
                        })
                );




        // G1 - Retract deposit
//        gamepadEx.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(
//                        new ConditionalCommand(
//                                new DepositRetractionCommand(),
//                                new InstantCommand(() -> CommandScheduler.getInstance().schedule(new DepositExtendCommand())),
//                                () -> Globals.IS_SCORING
//                        )
//                );

        // G1 - Claw control for scoring, retraction for when intaking
//        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(
//                        () -> CommandScheduler.getInstance().schedule(
//                                new ConditionalCommand(
//                                        new ClawDepositCommand(),
//                                        new ConditionalCommand(
//                                                new IntakeRetractCommand(),
//                                                new IntakeExtendCommand(extendIntake ? 500 : 100),
//                                                () -> Globals.IS_INTAKING
//                                        ),
//                                        () -> Globals.IS_SCORING
//                                ))
//                );
//
//        gamepadEx.getGamepadButton(GamepadKeys.Button.X)
//                .whenPressed(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> extendIntake = !extendIntake)));

        robot.read();
        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
        }
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        robot.clearBulkCache();
        robot.read();
        robot.periodic();
        robot.write();

//        // G1 - Drivetrain Control
//        robot.drivetrain.set(
//                new Pose(
//                        gamepad1.left_stick_x,
//                        -gamepad1.left_stick_y,
//                        MathUtils.joystickScalar(-gamepad1.left_trigger + gamepad1.right_trigger, 0.01)
//                ), 0
//        );

        robot.drivetrain.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, gamepad1.right_stick_x);
        telemetry.addData("left_stick_y", -gamepad1.left_stick_y);
        telemetry.addData("left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);


        // add if statement
        robot.hangLeftMotor.setPower(gamepad2.left_trigger);
        robot.hangLeftMotor.setPower(gamepad2.right_trigger);

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
//        telemetry.addData("height", robot.extension.getBackdropHeight());
        loopTime = loop;
        telemetry.update();
    }


}