package org.firstinspires.ftc.teamcode.tuning;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.common.vision.PropPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

public final class SplineTest extends LinearOpMode {

    RobotHardware robot;
    PropPipeline propPipeline;
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
                lift.getMotor().setPower(0.8);
                initialized = true;
            }

            // checks lift's current position
            double pos = lift.getMotor().getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos < 1400.0) {
                // true causes the action to rerun
                return true;
            } else {
                // false stops action rerun
                lift.getMotor().setPower(0);
                return false;
            }
            // overall, the action powers the lift until it surpasses
            // 3000 encoder ticks, then powers it off
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-34, 60, Math.toRadians(90));
        robot = RobotHardware.getInstance();
        propPipeline = new PropPipeline();
        lift = new LiftSubsystem();
        intake = new IntakeSubsystem();
        Globals.ALLIANCE = Location.BLUE;
        Globals.SIDE = Location.FAR;
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webka"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(propPipeline)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

//        while (robot.getCameraState() != VisionPortal.CameraState.STREAMING && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addLine("initializing... please wait");
//            telemetry.update();
//        }

        while (opModeInInit()) {
            telemetry.addLine("ready");
            telemetry.addData("position", propPipeline.getLocation());
            telemetry.update();
        }

        randomization = propPipeline.getLocation();
        portal.close();
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            blueFar(drive, randomization);

        } else {
            throw new RuntimeException();
        }
    }

    private static void blueNear(MecanumDrive drive) {
        TrajectoryActionBuilder trajBackdrop, trajStart;
        int propPosition = 2;
        switch(propPosition){
            case 0: // right
                trajStart = drive
                        .actionBuilder(new Pose2d(12, 60, Math.toRadians(90)))
                        .strafeToLinearHeading(new Vector2d(10, 34), 0);
                trajBackdrop = drive.actionBuilder(new Pose2d(10, 34, 0))
                        .strafeToLinearHeading(new Vector2d(48, 33), Math.toRadians(180));
                break;
            case 1: // center
                trajStart = drive
                        .actionBuilder(new Pose2d(12, 60, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(10, 34));
                trajBackdrop = drive.actionBuilder(new Pose2d(10, 34, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(20, 33))
                        .strafeToLinearHeading(new Vector2d(48, 33), Math.toRadians(180));

                break;
            case 2: // left
                trajStart = drive
                        .actionBuilder(new Pose2d(12, 60, Math.toRadians(90)))
                        .strafeToLinearHeading(new Vector2d(14, 34), Math.toRadians(180));
                trajBackdrop = drive.actionBuilder(new Pose2d(14, 34, Math.toRadians(180)))
                        .strafeToLinearHeading(new Vector2d(20, 45), Math.toRadians(135))
                        .strafeToLinearHeading(new Vector2d(48, 33), Math.toRadians(180));

                break;
            default:
                throw new Error("Unknown team prop position");
        }

        TrajectoryActionBuilder fTraj = drive.actionBuilder(new Pose2d(48, 33, Math.PI))
                .strafeToConstantHeading(new Vector2d(12, 60))
                .strafeToConstantHeading(new Vector2d(-48, 60))
                .strafeToConstantHeading(new Vector2d(-55, 34));
        TrajectoryActionBuilder fTrajEnd = drive.actionBuilder(new Pose2d(-55, 34, Math.PI))
                .strafeToConstantHeading(new Vector2d(-48, 60))
                .strafeToConstantHeading(new Vector2d(12, 60))
                .strafeToConstantHeading(new Vector2d(48, 33));

        Actions.runBlocking(new SequentialAction(
                trajStart.build(),
                trajBackdrop.build(),
                fTraj.build(),
                fTrajEnd.build()
        ));

    }

    private static void blueFar(MecanumDrive drive, Location loc) {

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
                trajBackdrop.build()
        ));

    }

    private static void redNear(MecanumDrive drive) {

        TrajectoryActionBuilder trajBackdrop, trajStart;
        int propPosition = 2;
        switch(propPosition){
            case 0: // left
                trajStart = drive
                        .actionBuilder(new Pose2d(12, -60, Math.toRadians(270)))
                        .strafeToLinearHeading(new Vector2d(14, -34), Math.toRadians(180));
                trajBackdrop = drive.actionBuilder(new Pose2d(14, -34, Math.toRadians(180)))
                        .strafeToConstantHeading(new Vector2d(14, -50))
                        .strafeToConstantHeading(new Vector2d(35, -44))
                        .strafeToConstantHeading(new Vector2d(48, -33));
                break;
            case 1: // center
                trajStart = drive
                        .actionBuilder(new Pose2d(12, -60, Math.toRadians(270)))
                        .strafeToConstantHeading(new Vector2d(12, -34))
                ;
                trajBackdrop = drive.actionBuilder(new Pose2d(12, -34, Math.toRadians(270)))
                        .strafeToConstantHeading(new Vector2d(20, -33))
                        .strafeToLinearHeading(new Vector2d(48, -33), Math.toRadians(180))
                ;
                break;
            case 2: // left
                trajStart = drive
                        .actionBuilder(new Pose2d(12, -60, Math.toRadians(270)))
                        .strafeToLinearHeading(new Vector2d(14, -34), 0);
                trajBackdrop = drive.actionBuilder(new Pose2d(14, -34, 0))
                        .strafeToLinearHeading(new Vector2d(48, -33), Math.toRadians(180));

                break;
            default:
                throw new Error("Unknown team prop position");
        }

        Actions.runBlocking(new SequentialAction(
                trajStart.build(),
                trajBackdrop.build()
        ));
    }


    private static void redFar(MecanumDrive drive) {

        TrajectoryActionBuilder trajStart, trajBackdrop;
        int propPosition = 2;

        switch(propPosition){
            case 0: // right
                trajStart = drive.actionBuilder(new Pose2d(-34, -60, Math.toRadians(270)))
                        .strafeToLinearHeading(new Vector2d(-36, -34), Math.toRadians(180));
                trajBackdrop = drive.actionBuilder(new Pose2d(-36, -36, Math.toRadians(180)))
                        .strafeToConstantHeading(new Vector2d(-34, -60))
                        .strafeToConstantHeading(new Vector2d(10, -60))
                        .strafeToConstantHeading(new Vector2d(48, -33));
                break;
            case 1: // center
                trajStart = drive.actionBuilder(new Pose2d(-34, -60, Math.toRadians(270)))
                        .strafeToConstantHeading(new Vector2d(-34, -34));
                trajBackdrop = drive.actionBuilder(new Pose2d(-34, -34, Math.toRadians(270)))
                        .strafeToConstantHeading(new Vector2d(-34, -60))
                        .strafeToConstantHeading(new Vector2d(10, -60))
                        .strafeToSplineHeading(new Vector2d(48, -33), Math.toRadians(180));
                break;
            case 2: // left
                trajStart = drive.actionBuilder(new Pose2d(-34, -60, Math.toRadians(270)))
                        .strafeToLinearHeading(new Vector2d(-34, -34), Math.toRadians(0));
                trajBackdrop = drive.actionBuilder(new Pose2d(-34, -34, Math.toRadians(0)))
                        .strafeToConstantHeading(new Vector2d(-34, -58))
                        .strafeToConstantHeading(new Vector2d(12, -58))
                        .strafeToSplineHeading(new Vector2d(48, -33), Math.PI);
                break;
            default:
                throw new Error("Unknown team prop position");
        }

        Actions.runBlocking(new SequentialAction(
                trajStart.build(),
                trajBackdrop.build()
        ));

    }

}
