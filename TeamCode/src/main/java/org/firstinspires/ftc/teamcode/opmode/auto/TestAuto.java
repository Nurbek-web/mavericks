package org.firstinspires.ftc.teamcode.opmode.auto;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.vision.VisionPortal;


@Config
@Autonomous(name = "TestAuto", group = "Autonomous")
public class TestAuto extends LinearOpMode {
    RobotHardware robot;
//    PropPipeline propPipeline;
    VisionPortal portal;
    Location randomization;

    LiftSubsystem lift;
    IntakeSubsystem intake;

    public class LiftUp implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                robot.liftMotor.setPower(0.8);
                initialized = true;
            }

            // checks lift's current position
            double pos = robot.liftMotor.getCurrentPosition();
            packet.put("liftPos", pos);

            if (pos < 1400) {
                // true causes the action to rerun
                return true;
            } else {
                // false stops action rerun
                robot.liftMotor.setPower(0);
                return false;
            }
            // overall, the action powers the lift until it surpasses
            // 3000 encoder ticks, then powers it off
        }
    }


    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-34, 60, Math.toRadians(90)));

        // vision here that outputs position
        int visionOutputPosition = 1;

        Action trajectoryAction1;


        waitForStart();

        blueFar(drive, Location.LEFT);

    }

    private void blueFar(MecanumDrive drive, Location loc) {

        TrajectoryActionBuilder trajStart, trajBackdrop;

        switch(loc){
            case RIGHT: // right
                trajStart = drive.actionBuilder(new Pose2d(-34, 60, Math.toRadians(90)))
                        .strafeToLinearHeading(new Vector2d(-38, 45), Math.toRadians(45))
                        .strafeToLinearHeading(new Vector2d(-36, 34), Math.toRadians(0));
                trajBackdrop = drive.actionBuilder(new Pose2d(-36, 34, Math.toRadians(0)))
                        .strafeToConstantHeading(new Vector2d(-34, 58))
                        .strafeToConstantHeading(new Vector2d(12, 58))
                        .strafeToLinearHeading(new Vector2d(48, 33), Math.toRadians(180));
                break;
            case CENTER: // center
                trajStart = drive.actionBuilder(new Pose2d(-34, 60, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(-34, 34));
                trajBackdrop = drive.actionBuilder(new Pose2d(-34, 34, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(-34, 58))
                        .strafeToConstantHeading(new Vector2d(12, 58))
                        .strafeToLinearHeading(new Vector2d(48, 33), Math.toRadians(180));
                break;
            case LEFT: // left
                trajStart = drive.actionBuilder(new Pose2d(-34, 60, Math.toRadians(90)))
                        .lineToY(46).strafeToLinearHeading(new Vector2d(-34, 30), Math.toRadians(180));
                trajBackdrop = drive.actionBuilder(new Pose2d(-34, 30, Math.toRadians(180)))
                        .strafeToConstantHeading(new Vector2d(-34, 58))
                        .strafeToConstantHeading(new Vector2d(12, 58))
                        .strafeToConstantHeading(new Vector2d(48, 33));
                break;
            default:
                throw new Error("Unknown team prop position");
        }

        Actions.runBlocking(new SequentialAction(
                trajStart.build(),
                new ParallelAction(
                        trajBackdrop.build(),
                        new LiftUp()
                )

        ));

    }

}