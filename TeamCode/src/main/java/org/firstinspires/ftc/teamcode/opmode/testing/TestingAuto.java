//package org.firstinspires.ftc.teamcode.opmode.testing;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
//import org.firstinspires.ftc.teamcode.common.vision.BlueProcessor;
//import org.firstinspires.ftc.teamcode.opmode.auto.BlueCloseAutoVPortal;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//
//@Autonomous
//public class TestingAuto extends LinearOpMode {
//    private final RobotHardware robot = RobotHardware.getInstance();
//
//    private BlueProcessor processor;
//    private VisionPortal visionPortal;
//
//    MecanumDrive drive;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        drive = new MecanumDrive(hardwareMap, new Pose2d(12, 60, Math.toRadians(90)));
//        robot.init(hardwareMap);
//
//        // Create the AprilTag processor the easy way.
//        processor = BlueProcessor.easyCreateWithDefaults();
//
//        telemetry.addLine("Inited");
//        telemetry.update();
//
//        waitForStart();
//
//        telemetry.addLine("STARTED");
//        telemetry.update();
//
//        TrajectoryActionBuilder trajStart = drive
//                .actionBuilder(new Pose2d(12, 60, Math.toRadians(90)))
//
//                .strafeToConstantHeading(new Vector2d(12, 35.5))
//                .turn(-Math.toRadians(90))
//                .strafeToConstantHeading(new Vector2d(8, 35.5))
//                .strafeToConstantHeading(new Vector2d(12, 35.5));
//        Actions.runBlocking(new SequentialAction(
//                trajStart.build()
//        ));
//    }
//}
