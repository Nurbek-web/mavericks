
package org.firstinspires.ftc.teamcode.common.commandbase.teleopcommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.common.subsystem.HangSubsystem;
//
//public class ActuateHangCommand extends ConditionalCommand {
//    public ActuateHangCommand(double value) {
//        super(
//                new SequentialCommandGroup(
////                        new InstantCommand(() -> RobotHardware.getInstance().hangRight.setPosition(0))
//                        new InstantCommand(() -> RobotHardware.getInstance().hangRight.setPosition(0))
////                        new InstantCommand(() -> RobotHardware.getInstance().hangLeft.setPower(value))
//                ),
//                new WaitCommand(0),
//
//        );
//    }
//}
