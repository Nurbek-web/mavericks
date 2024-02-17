package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-34, 60, Math.toRadians(90));
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            blueFar(drive);

        } else {
            throw new RuntimeException();
        }
    }

    private static void blueNear(MecanumDrive drive) {
        Action trajStart = drive.actionBuilder(new Pose2d(12, 60, Math.toRadians(270)))
                .lineToY(34)
//                .waitSeconds(0.4)
//                .turn(Math.toRadians(-90))
//                .lineToX(50)
                .build();
        int propPosition = 2;
        TrajectoryActionBuilder trajPropBuilder = drive.
                actionBuilder(new Pose2d(12, 34, Math.toRadians(270)));
        Action trajProp, reverseTrajProp;
        switch(propPosition){
            case 0: // left
                trajProp = trajPropBuilder.turn(-Math.PI/2).build();
                reverseTrajProp = drive.
                        actionBuilder(new Pose2d(12, 34, Math.toRadians(180))).
                        turn(Math.toRadians(-90)).build();
                break;
            case 1: // center
                trajProp = trajPropBuilder.turn(Math.PI).build();
                reverseTrajProp = drive.
                        actionBuilder(new Pose2d(12, 34, Math.toRadians(90)))
                        .turn(Math.toRadians(0)).build();
                break;
            case 2: // right
                trajProp = trajPropBuilder.turn(Math.PI/2).build();
                reverseTrajProp = drive.
                        actionBuilder(new Pose2d(12, 34, 0.0)).
                        turn(-Math.toRadians(-90)).build();
                break;
            default:
                throw new Error("Unknown team prop position");
        }
        Action trajGoToBackdrop = drive.actionBuilder(new Pose2d(12, 34, Math.toRadians(90)))
                .splineTo(new Vector2d(25, 47), 0)
                .splineTo(new Vector2d(50, 33), 0)
                .build();
        Actions.runBlocking(new SequentialAction(
                trajStart,
                trajProp,

                // servo action
                reverseTrajProp,
                trajGoToBackdrop
        ));

    }

    private static void blueFar(MecanumDrive drive) {

        TrajectoryActionBuilder trajStart, trajBackdrop;
        int propPosition = 2;

        switch(propPosition){
            case 0: // right
                trajStart = drive.actionBuilder(new Pose2d(-34, 60, Math.toRadians(90)))
                        .strafeToLinearHeading(new Vector2d(-38, 45), Math.toRadians(45))
                        .strafeToLinearHeading(new Vector2d(-36, 34), Math.toRadians(0));
                trajBackdrop = drive.actionBuilder(new Pose2d(-36, 34, Math.toRadians(0)))
                        .strafeToConstantHeading(new Vector2d(-34, 60))
                        .strafeToConstantHeading(new Vector2d(12, 60))
                        .strafeToLinearHeading(new Vector2d(48, 33), Math.toRadians(180));
                break;
            case 1: // center
                trajStart = drive.actionBuilder(new Pose2d(-34, 60, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(-34, 34));
                trajBackdrop = drive.actionBuilder(new Pose2d(-34, 34, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(-34, 60))
                        .strafeToConstantHeading(new Vector2d(12, 60))
                        .strafeToLinearHeading(new Vector2d(48, 33), Math.toRadians(180));
                break;
            case 2: // left
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
