package org.firstinspires.ftc.teamcode.opmode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.vision.sim.BasicPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name = "BlueCloseAutoVPortal", group = "Autonomous")
public class BlueCloseAutoVPortal extends LinearOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    BasicPipeline pipeline = new BasicPipeline();
    OpenCvCamera camera;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public Servo autoServo;
    //    PropPipeline propPipeline;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    Location randomization;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    private ElapsedTime runtime = new ElapsedTime();

    // UNITS ARE METERS
    //Converted from 2 inches
    //TODO: To be tested (tag size)
    double tagsize = 0.051;
    Location givenTag;

    LiftSubsystem lift;
    IntakeSubsystem intake;
    Location loc;
    MecanumDrive drive;

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "webka"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()


    public class Sleep implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            autoServo.setPosition(0.68);
            sleep(4000);
            return false;
        }
    }

    public class AprilTagAlign implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            boolean tagFound = false;
            camera.closeCameraDevice();

            initAprilTag();
            AprilTagDetection tagOfInterest = null;

            List<AprilTagDetection> currentDetections;
            runtime.reset();
            telemetry.addLine("preStart");
            telemetry.update();
            while (!tagFound && runtime.seconds() <= 5.0) {
                currentDetections = aprilTag.getDetections();

                for (AprilTagDetection tag : currentDetections) {

                    if (tag.metadata != null) {
                        tagFound = true;
                        tagOfInterest = tag;
                        break;
                    }
                }
            }
            if (tagFound) {
                drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

                double x = tagOfInterest.ftcPose.x - 0.095;
                double y = tagOfInterest.ftcPose.y - 11.05;

                telemetry.addData("x", x);
                telemetry.addData("y", y);

                telemetry.update();

                TrajectoryActionBuilder trajApril, traj2;
                trajApril = drive.actionBuilder(new Pose2d(0, 0, 0))
                        .turn(Math.toRadians(tagOfInterest.ftcPose.yaw));


                Actions.runBlocking(new SequentialAction(
                        trajApril.build()
                ));

                currentDetections = aprilTag.getDetections();

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.metadata != null) {
                        if ((loc == Location.LEFT && (tag.id == 1 || tag.id == 4))
                                || (loc == Location.CENTER && (tag.id == 2 || tag.id == 5))
                                || (loc == Location.RIGHT && (tag.id == 3 || tag.id == 6))) {
                            tagFound = true;
                            tagOfInterest = tag;
                            break;
                        }
                    }
                }

                x = tagOfInterest.ftcPose.x - 0.095;
                y = tagOfInterest.ftcPose.y - 11.2;

                telemetry.addData("x", x);
                telemetry.addData("y", y);

                telemetry.update();


                if (loc == Location.LEFT) {
                    traj2 = drive.actionBuilder(new Pose2d(0, 0, 0))
                            .strafeToConstantHeading(new Vector2d(-y, x - 1.8));
                } else {
                    traj2 = drive.actionBuilder(new Pose2d(0, 0, 0))
                            .strafeToConstantHeading(new Vector2d(-y, x));
                }


                Actions.runBlocking(new SequentialAction(
                        traj2.build()
                ));
                drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

                Actions.runBlocking(new SequentialAction(
                        new LiftUp(),
                        drive.actionBuilder(new Pose2d(0, 0, 0))
                                .strafeToConstantHeading(new Vector2d(0, -20)).build()
                ));
                visionPortal.close();
                return false;

            } else {

                telemetry.addLine("NO APRIL TAG");
                telemetry.update();
            }
                drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            Actions.runBlocking(new SequentialAction(
                    new LiftUp(),
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .strafeToConstantHeading(new Vector2d(0, -20)).build()
            ));
            return false;
        }
    }


    public class DropPixel implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            autoServo.setPosition(0.68);
            sleep(1000);
            return false;
        }
    }

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

            if (pos < 885) {
                // true causes the action to rerun
                return true;
            } else {
                // false stops action rerun
                robot.liftMotor.setPower(0);
                lift.extend1Outtake();
                sleep(1400);
                lift.openOuttake();

                sleep(2000);
                return false;
            }
            // overall, the action powers the lift until it surpasses
            // 3000 encoder ticks, then powers it off
        }
    }


    @Override
    public void runOpMode() {
        Globals.IS_AUTO = true;
        // vision here that outputs position
        sleep(1000);
        int visionOutputPosition = 1;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webka"), cameraMonitorViewId);
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });



        telemetry.update();
        robot.init(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, 60, Math.toRadians(90)));
        lift = new LiftSubsystem();

        autoServo = hardwareMap.get(Servo.class, "autoServo");
        robot.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        autoServo.setPosition(1);
        lift.closeOuttake();
        lift.intendOuttake();

        waitForStart();

        sleep(2000);

        if (pipeline.getJunctionPoint().x < 330) {
            telemetry.addLine("LEFT PROP");
            loc = Location.LEFT;
        }
        else if (pipeline.getJunctionPoint().x > 950) {
            loc = Location.RIGHT;
            telemetry.addLine("RIGHT PROP");
        } else {
            loc = Location.CENTER;
            telemetry.addLine("CENTER PROP");
        }

        telemetry.addLine("STARTED");
        redClose(drive, loc);

    }

    private void redClose(MecanumDrive drive, Location loc) {

        TrajectoryActionBuilder trajStart, trajBackdrop;

        Vector2d startingPosition;
        switch(loc){
            case RIGHT: // right
                trajStart = drive
                        .actionBuilder(new Pose2d(12, 60, Math.toRadians(90)))

                        .strafeToConstantHeading(new Vector2d(12, 35.5))
                        .turn(-Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(8, 35.5))
                        .strafeToConstantHeading(new Vector2d(12, 35.5));
                trajBackdrop = drive.actionBuilder(new Pose2d(12, 35.5, 0))
                        .splineToSplineHeading(new Pose2d(37.8, 30, Math.PI), 0);

//                        .strafeToConstantHeading(new Vector2d(14, 50))
//                        .strafeToConstantHeading(new Vector2d(35, 50))
//                        .strafeToConstantHeading(new Vector2d(38, 41));
                startingPosition = new Vector2d(37.8, 30);
                break;
            case CENTER: // center
                trajStart = drive.actionBuilder(new Pose2d(12, 60, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(12, 30))
                        .strafeToConstantHeading(new Vector2d(12, 36));
                trajBackdrop = drive.actionBuilder(new Pose2d(12, 36, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(30, 33))
                        .splineToSplineHeading(new Pose2d(41.7, 39.5, Math.PI), 0);
//                        .splineToSplineHeading(new Pose2d(38, -34.5, Math.PI), 0);
                startingPosition = new Vector2d(37.8, 35.5);

                break;
            case LEFT: // left
                trajStart = drive
                        .actionBuilder(new Pose2d(12, 60, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(14, 34))
                        .turn(Math.toRadians(90));
//                        .strafeToConstantHeading(new Vector2d(18, 34))
//                        .strafeToConstantHeading(new Vector2d(14, 34));
                trajBackdrop = drive.actionBuilder(new Pose2d(14, 34, Math.toRadians(180)))
                        .strafeToLinearHeading(new Vector2d(20, 45), Math.toRadians(135))
                        .strafeToLinearHeading(new Vector2d(41.7, 45.5), Math.toRadians(180));
//                        .splineToSplineHeading(new Pose2d(37.8, -29, Math.PI), 0);

                break;
            default:
                throw new Error("Unknown team prop position");
        }

        Actions.runBlocking(new SequentialAction(
                trajStart.build(),
                new DropPixel(),
                trajBackdrop.build(),
                new AprilTagAlign()
        ));
        robot.kill();
    }



}