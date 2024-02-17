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
        Pose2d beginPose = new Pose2d(12, 60, Math.toRadians(90));
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            blueFar(drive);
////            Actions.runBlocking(
////                drive.actionBuilder(beginPose)
////                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
////                        .splineTo(new Vector2d(-30, -30), Math.PI / 2)
////                        .build());
//
//            Action trajStart = drive.actionBuilder(new Pose2d(12, 60, Math.toRadians(90)))
//                    .lineToY(34)
////                .waitSeconds(0.4)
////                .turn(Math.toRadians(-90))
////                .lineToX(50)
//                    .build();
//            int propPosition = 1;
//            TrajectoryActionBuilder trajPropBuilder = drive.
//                    actionBuilder(new Pose2d(12, 34, Math.toRadians(90)));
//            Action trajProp, reverseTrajProp;
//            switch(propPosition){
//                case 0: // left
//                    trajProp = trajPropBuilder.turn(-Math.PI/2).build();
//                    reverseTrajProp = drive.
//                            actionBuilder(new Pose2d(12, 34, Math.toRadians(0))).
//                            turn(Math.toRadians(-90)).build();
//                    break;
//                case 1: // center
//                    trajProp = trajPropBuilder.build();
//                    reverseTrajProp = drive.
//                            actionBuilder(new Pose2d(12, 34, Math.toRadians(90)))
////                        .turn(Math.toRadians(180))
//                            .build();
//                    break;
//                case 2: // right
//                    trajProp = trajPropBuilder.turn(Math.PI/2).waitSeconds(1).build();
//                    reverseTrajProp = drive.
//                            actionBuilder(new Pose2d(12, 34, 180)).waitSeconds(1).
//                            turn(Math.toRadians(90)).waitSeconds(1).build();
//                    break;
//                default:
//                    throw new Error("Unknown team prop position");
//            }
//            Action trajGoToBackdrop = drive.actionBuilder(new Pose2d(12, 34, Math.toRadians(90)))
////                .splineTo(new Vector2d(25, 47), 180)
//                    .strafeToLinearHeading(new Vector2d(50, 33), Math.toRadians(180))
//                    .build();
//            Actions.runAction(new SequentialAction(
//                    trajStart,
//                    trajProp,
//
//                    // servo action
//                    reverseTrajProp,
//                    trajGoToBackdrop
//            ));


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

        Action trajStart = drive.actionBuilder(new Pose2d(-34, 60, Math.toRadians(270)))
                .lineToY(34)
//                .waitSeconds(0.4)
//                .turn(Math.toRadians(-90))
//                .lineToX(50)
                .build();
        int propPosition = 2;
        TrajectoryActionBuilder trajPropBuilder = drive.
                actionBuilder(new Pose2d(-34, 60, Math.toRadians(270)));
        Action trajProp, reverseTrajProp;
        switch(propPosition){
            case 0: // left
                trajProp = trajPropBuilder.turn(-Math.PI/2).build();
                reverseTrajProp = drive.
                        actionBuilder(new Pose2d(-34, 34, Math.toRadians(180))).
                        turn(Math.toRadians(-90)).build();
                break;
            case 1: // center
                trajProp = trajPropBuilder.turn(Math.PI).build();
                reverseTrajProp = drive.
                        actionBuilder(new Pose2d(-34, 34, Math.toRadians(90)))
                        .turn(Math.toRadians(0)).build();
                break;
            case 2: // right
                trajProp = trajPropBuilder.turn(Math.PI/2).build();
                reverseTrajProp = drive.
                        actionBuilder(new Pose2d(-34, 34, Math.PI)).
                        turn(Math.toRadians(90)).build();
                break;
            default:
                throw new Error("Unknown team prop position");
        }
        Action trajGoToBackdrop = drive.actionBuilder(new Pose2d(-34, 34, Math.toRadians(90)))
                .splineTo(new Vector2d(-28, 60), 0)
                .lineToX(25)
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

    private static void redNear(MecanumDrive drive) {

        Action trajStart = drive.actionBuilder(new Pose2d(12, -60, Math.toRadians(90)))
                .lineToY(-34)
//                .waitSeconds(0.4)
//                .turn(Math.toRadians(-90))
//                .lineToX(50)
                .build();
        int propPosition = 2;
        TrajectoryActionBuilder trajPropBuilder = drive.
                actionBuilder(new Pose2d(12, -34, Math.toRadians(90)));
        Action trajProp, reverseTrajProp;
        switch(propPosition){
            case 0: // left
                trajProp = trajPropBuilder.turn(-Math.PI/2).build();
                reverseTrajProp = drive.
                        actionBuilder(new Pose2d(12, -34, Math.toRadians(0))).
                        turn(Math.toRadians(-90)).build();
                break;
            case 1: // center
                trajProp = trajPropBuilder.turn(Math.PI).build();
                reverseTrajProp = drive.
                        actionBuilder(new Pose2d(12, 34, Math.toRadians(270)))
                        .build();
                break;
            case 2: // right
                trajProp = trajPropBuilder.turn(Math.PI/2).build();
                reverseTrajProp = drive.
                        actionBuilder(new Pose2d(12, -34, Math.PI)).
                        turn(Math.toRadians(90)).build();
                break;
            default:
                throw new Error("Unknown team prop position");
        }
        Action trajGoToBackdrop = drive.actionBuilder(new Pose2d(12, -34, Math.toRadians(270)))
                .splineTo(new Vector2d(25, -47), 0)
                .splineTo(new Vector2d(50, -33), 0)
                .build();
        Actions.runBlocking(new SequentialAction(
                trajStart,
                trajProp,

                // servo action
                reverseTrajProp,
                trajGoToBackdrop
        ));

    }

    private static void redFar(MecanumDrive drive) {

        Action trajStart = drive.actionBuilder(new Pose2d(-34, -60, Math.toRadians(90)))
                .lineToY(-34)
                .build();
        int propPosition = 2;
        TrajectoryActionBuilder trajPropBuilder = drive.
                actionBuilder(new Pose2d(-34, -34, Math.toRadians(90)));
        Action trajProp, reverseTrajProp;
        switch(propPosition){
            case 0: // left
                trajProp = trajPropBuilder.turn(-Math.PI/2).build();
                reverseTrajProp = drive.
                        actionBuilder(new Pose2d(-34, -34, Math.toRadians(0))).
                        turn(Math.toRadians(-90)).build();
                break;
            case 1: // center
                trajProp = trajPropBuilder.turn(Math.PI).build();
                reverseTrajProp = drive.
                        actionBuilder(new Pose2d(-34, -34, Math.toRadians(270)))
                        .build();
                break;
            case 2: // right
                trajProp = trajPropBuilder.turn(Math.PI/2).build();
                reverseTrajProp = drive.
                        actionBuilder(new Pose2d(-34, -34, Math.PI)).
                        turn(Math.toRadians(90)).build();
                break;
            default:
                throw new Error("Unknown team prop position");
        }
        Action trajGoToBackdrop = drive.actionBuilder(new Pose2d(-34, -34, Math.toRadians(270)))
                .splineTo(new Vector2d(-28, -60), 0)
                .lineToX(25)
                .splineTo(new Vector2d(50, -33), 0)
                .build();
        Actions.runBlocking(new SequentialAction(
                trajStart,
                trajProp,

                // servo action
                reverseTrajProp,
                trajGoToBackdrop
        ));
    }

}
