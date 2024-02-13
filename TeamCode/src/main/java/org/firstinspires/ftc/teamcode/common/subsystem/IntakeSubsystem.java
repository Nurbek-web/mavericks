package org.firstinspires.ftc.teamcode.common.subsystem;


import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

public class IntakeSubsystem extends SubsystemBase {

    private final RobotHardware robot;

    public IntakeSubsystem() {
        this.robot = RobotHardware.getInstance();
    }

    public void runIntake(){
        if(Globals.IS_INTAKING==Globals.IntakeState.INTAKING || !Globals.INTAKE_LOWERED){
            return;
        }
        this.robot.intakeRoller.setPower(0.8);
        this.robot.intakeMotor.setPower(0.8);
        Globals.startIntaking();
    }

    public void stopIntake(){
        if(Globals.IS_INTAKING==Globals.IntakeState.NOT_WORKING){
            return;
        }
        this.robot.intakeRoller.setPower(0);
        this.robot.intakeMotor.setPower(0);
        Globals.stopIntaking();
    }

    public void lowerServo(){
        if(Globals.INTAKE_LOWERED){
            return;
        }
        this.robot.intakeServo.setPosition(0.5);
        Globals.lowerIntake();
    }
    public void raiseServo(){
        if(!Globals.INTAKE_LOWERED){
            return;
        }
        this.robot.intakeServo.setPosition(0);
        Globals.raiseIntake();
    }

    public void releaseExtra(){
        if(Globals.IS_INTAKING==Globals.IntakeState.REVERSE || !Globals.INTAKE_LOWERED){
            return;
        }
        this.robot.intakeMotor.setPower(-0.8);
        this.robot.intakeRoller.setPower(-0.8);
        Globals.releaseIntaking();
    }
}


//public class IntakeSubsystem extends WSubsystem {
//
//    private final RobotHardware robot;
//
//    private PivotState pivotState;
//
//    public ClawState leftClaw = ClawState.CLOSED;
//    public ClawState rightClaw = ClawState.CLOSED;
//
//    public enum ClawState {
//        CLOSED,
//        INTERMEDIATE,
//        OPEN
//    }
//
//    public enum PivotState {
//        FLAT,
//        STORED,
//        SCORING
//    }
//
//    public IntakeSubsystem() {
//        this.robot = RobotHardware.getInstance();
//
//        updateState(ClawState.CLOSED, ClawSide.BOTH);
//    }
//
//    public void updateState(@NotNull ClawState state, @NotNull ClawSide side) {
//        double position = getClawStatePosition(state, side);
//        switch (side) {
//            case LEFT:
//                robot.intakeClawLeftServo.setPosition(position);
//                this.leftClaw = state;
//                break;
//            case RIGHT:
//                robot.intakeClawRightServo.setPosition(position);
//                this.rightClaw = state;
//                break;
//            case BOTH:
//                robot.intakeClawLeftServo.setPosition(getClawStatePosition(state, ClawSide.LEFT));
//                this.leftClaw = state;
//                robot.intakeClawRightServo.setPosition(getClawStatePosition(state, ClawSide.RIGHT));
//                this.rightClaw = state;
//                break;
//        }
//    }
//
//    public void updateState(@NotNull PivotState state) {
//        this.pivotState = state;
//    }
//
//    @Override
//    public void periodic() {
//        double pos = robot.armActuator.getOverallTargetPosition();
//        if (pivotState == PivotState.SCORING) {
//            double targetAngle = (pos) - (((pos < Math.PI / 2) ? 0.22 : 0.78) * Math.PI);
//            robot.intakePivotActuator.setTargetPosition(MathUtils.clamp(MathUtils.map(targetAngle, 0, Math.PI / 2 - 0.35, 0.5, 0.93), 0.03, 0.97));
//        } else if (pivotState == PivotState.FLAT) {
////            double targetAngle = ((pos) - ((((pos < Math.PI / 2) ? 0 : 1) * Math.PI)));
////            robot.intakePivotActuator.setTargetPosition(MathUtils.clamp(MathUtils.map(targetAngle, 0, Math.PI / 2 - 0.35, 0.5, 0.93) + 0.03, 0.03, 0.97));
//        } else if (pivotState == PivotState.STORED) {
//            robot.intakePivotActuator.setTargetPosition(0.05);
//        }
//    }
//
//    @Override
//    public void read() {
//
//    }
//
//    @Override
//    public void write() {
//        robot.intakePivotActuator.write();
//    }
//
//    @Override
//    public void reset() {
//        updateState(PivotState.STORED);
//    }
//
//    private double getClawStatePosition(ClawState state, ClawSide side) {
//        switch (side) {
//            case LEFT:
//                switch (state) {
//                    case CLOSED:
//                        return 0.07;
//                    case INTERMEDIATE:
//                        return 0.17;
//                    case OPEN:
//                        return 0.38;
//                    default:
//                        return 0.0;
//                }
//            case RIGHT:
//                switch (state) {
//                    case CLOSED:
//                        return 0.52;
//                    case INTERMEDIATE:
//                        return 0.62;
//                    case OPEN:
//                        return 0.89;
//                    default:
//                        return 0.0;
//                }
//            default:
//                return 0.0;
//        }
//    }
//
//    public ClawState getClawState(ClawSide side) {
//        if (side == ClawSide.BOTH)
//            return (robot.intake.rightClaw == (IntakeSubsystem.ClawState.CLOSED) || (robot.intake.leftClaw == IntakeSubsystem.ClawState.CLOSED)) ? ClawState.CLOSED : ClawState.OPEN;
//        return (side == ClawSide.LEFT) ? leftClaw : rightClaw;
//    }
//}
