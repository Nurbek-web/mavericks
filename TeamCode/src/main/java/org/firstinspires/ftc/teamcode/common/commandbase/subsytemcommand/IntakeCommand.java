//package org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
//
//public class IntakeCommand extends CommandBase {
//    IntakeSubsystem.IntakeAction curAction;
//    IntakeSubsystem mSubsystem;
//    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
//        this.mSubsystem = intakeSubsystem;
//    }
//}


//    new ConditionalCommand(
//        new InstantCommand(intake::run, intake),
//        new InstantCommand(intake::stop, intake),
//        () -> {
//        intake.toggle();
//        return intake.active();
//    }
