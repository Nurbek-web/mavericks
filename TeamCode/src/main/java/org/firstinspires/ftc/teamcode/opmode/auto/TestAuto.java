//package org.firstinspires.ftc.teamcode.opmode.auto;
//
//import androidx.annotation.NonNull;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//
//@Config
//@Autonomous(name = "TestAuto", group = "Autonomous")
//public class TestAuto extends LinearOpMode {
//
//    @Override
//    public void runOpMode() {
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(90)));
//
//        // vision here that outputs position
//        int visionOutputPosition = 1;
//
//        Action trajectoryAction1;
//        Action trajectoryAction2;
//        Action trajectoryAction3;
//        Action trajectoryActionCloseOut;
//
//        trajectoryAction1 = drive.actionBuilder(drive.pose)
//                .lineToYSplineHeading(33, Math.toRadians(0))
//                .waitSeconds(2)
//                .setTangent(Math.toRadians(90))
//                .lineToY(48)
//                .setTangent(Math.toRadians(0))
//                .lineToX(32)
//                .strafeTo(new Vector2d(44.5, 30))
//                .turn(Math.toRadians(180))
//                .lineToX(47.5)
//                .waitSeconds(3)
//                .build();
//        trajectoryAction2 = drive.actionBuilder(drive.pose)
//                .lineToY(37)
//                .setTangent(Math.toRadians(0))
//                .lineToX(18)
//                .waitSeconds(3)
//                .setTangent(Math.toRadians(0))
//                .lineToXSplineHeading(46, Math.toRadians(180))
//                .waitSeconds(3)
//                .build();
//        trajectoryAction3 = drive.actionBuilder(drive.pose)
//                .lineToYSplineHeading(33, Math.toRadians(180))
//                .waitSeconds(2)
//                .strafeTo(new Vector2d(46, 30))
//                .waitSeconds(3)
//                .build();
//        trajectoryActionCloseOut = drive.actionBuilder(drive.pose)
//                .strafeTo(new Vector2d(48, 12))
//                .build();
//
//        // actions that need to happen on init; for instance, a claw tightening.
//
//
//        while (!isStopRequested() && !opModeIsActive()) {
//            int position = visionOutputPosition;
//            telemetry.addData("Position during Init", position);
//            telemetry.update();
//        }
//
//        int startPosition = visionOutputPosition;
//        telemetry.addData("Starting Position", startPosition);
//        telemetry.update();
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        Action trajectoryActionChosen;
//        if (startPosition == 1) {
//            trajectoryActionChosen = trajectoryAction1;
//        } else if (startPosition == 2) {
//            trajectoryActionChosen = trajectoryAction2;
//        } else {
//            trajectoryActionChosen = trajectoryAction3;
//        }
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        trajectoryActionChosen,
//                        trajectoryActionCloseOut
//                )
//        );
//    }
//}