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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.vision.sim.BasicPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Config
@Autonomous(name = "TestAuto", group = "Autonomous")
public class TestAuto extends LinearOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    BasicPipeline pipeline = new BasicPipeline();
    OpenCvCamera camera;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public Servo autoServo;
    //    PropPipeline propPipeline;
    VisionPortal portal;
    Location randomization;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    //Converted from 2 inches
    //TODO: To be tested (tag size)
    double tagsize = 0.051;

    LiftSubsystem lift;
    IntakeSubsystem intake;
    Location loc;
    public class Sleep implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            autoServo.setPosition(0.68);
            sleep(4000);
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

            if (pos < 1550) {
                // true causes the action to rerun
                return true;
            } else {
                // false stops action rerun
                robot.liftMotor.setPower(0);
                lift.extend1Outtake();
                sleep(1000);
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

        sleep(3500);

        if (pipeline.getJunctionPoint().x < 500) {
            telemetry.addLine("LEFT PROP");
            loc = Location.LEFT;
        }
        else if (pipeline.getJunctionPoint().x > 850) {
            loc = Location.RIGHT;
            telemetry.addLine("RIGHT PROP");
        } else {
            loc = Location.CENTER;
            telemetry.addLine("CENTER PROP");
        }

        telemetry.update();
        robot.init(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-34, 60, Math.toRadians(90)));
        lift = new LiftSubsystem();

        autoServo = hardwareMap.get(Servo.class, "autoServo");
        robot.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        autoServo.setPosition(1);
        lift.closeOuttake();
        lift.intendOuttake();

        waitForStart();

        telemetry.addLine("STARTED");
        blueFar(drive, Location.LEFT);

    }

    private void blueFar(MecanumDrive drive, Location loc) {

        TrajectoryActionBuilder trajStart, trajBackdrop;
        AprilTagPoseFtc aPose;

        switch(loc){
            case RIGHT: // right
                trajStart = drive.actionBuilder(new Pose2d(-34, 60, Math.toRadians(90)))
                        .strafeToLinearHeading(new Vector2d(-38, 45), Math.toRadians(45))
                        .strafeToLinearHeading(new Vector2d(-36, 34), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(-41, 34), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(-36, 34), Math.toRadians(0));
                trajBackdrop = drive.actionBuilder(new Pose2d(-36, 34, Math.toRadians(0)))
                        .strafeToConstantHeading(new Vector2d(-34, 57))
                        .strafeToConstantHeading(new Vector2d(12, 57))
                        .splineToLinearHeading(new Pose2d(37.8, 28.5, Math.toRadians(180)), 0);
                aPose = robot.getAprilTagPosition(Location.RIGHT);

                break;
            case CENTER: // center
                trajStart = drive.actionBuilder(new Pose2d(-34, 60, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(-34, 30))
                        .strafeToConstantHeading(new Vector2d(-34, 35));
                trajBackdrop = drive.actionBuilder(new Pose2d(-34, 35, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(-34, 57))
                        .strafeToConstantHeading(new Vector2d(12, 57))
                        .splineToLinearHeading(new Pose2d(37.8, 35.5, Math.toRadians(180)), 0);
                break;
            case LEFT: // left
                telemetry.addLine("LEFT");
                trajStart = drive.actionBuilder(new Pose2d(-34, 60, Math.toRadians(90)))
                        .lineToY(46).strafeToLinearHeading(new Vector2d(-34, 30), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(-27.5, 30), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(-34, 30), Math.toRadians(180));
                trajBackdrop = drive.actionBuilder(new Pose2d(
                                -34, 30, Math.toRadians(180)))
                        .strafeToConstantHeading(new Vector2d(-34, 57))
                        .strafeToConstantHeading(new Vector2d(12, 57))
                        .splineToConstantHeading(new Vector2d(37.8, 45.5), 0);
                Actions.runBlocking(new SequentialAction(
                                trajStart.build(),
                                new DropPixel(),
                                trajBackdrop.build()
                        )

                );

                AprilTagDetectionPipeline aprilTagDetectionPipeline;
                aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);


                org.openftc.apriltag.AprilTagDetection tagOfInterest = null;

                int TAG1 = 1;
                int TAG2 = 2;
                int TAG3 = 3; //dont reall need number 3 but like honestly its good to have it
                //if we need to make more or remove more remember to update the code on the bottom of this code

                camera.setPipeline(aprilTagDetectionPipeline);

                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
                while (true)
                {
                    boolean tagFound = false;
                    currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                    for(org.openftc.apriltag.AprilTagDetection tag : currentDetections)
                    {
                        if(tag.id == TAG1)
                        {
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }

                    if(tagFound)
                    {
                        telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                        tagToTelemetry(tagOfInterest);
                        break;
                    }
                    else
                    {
                        telemetry.addLine("Don't see tag of interest :(");

                        if(tagOfInterest == null)
                        {
                            telemetry.addLine("(The tag has never been seen)");
                        }
                        else
                        {
                            telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                            tagToTelemetry(tagOfInterest);
                        }
                    }

                }
                telemetry.update();
                sleep(20);

                /*
                 * The START command just came in: now work off the latest snapshot acquired
                 * during the init loop.
                 */

                /* Update the telemetry */
                if(tagOfInterest != null)
                {
                    telemetry.addLine("Tag snapshot:\n");
                    tagToTelemetry(tagOfInterest);
                    telemetry.update();
                }
                else {
                    telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                    telemetry.update();
                }

                drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

                double x = tagOfInterest.pose.x - 0.095;
                double xperInch = 40;

                if(tagOfInterest.id == TAG1)
                {

                    telemetry.addLine("TAG1");
                    telemetry.addData("FIRST: ", x);
                    telemetry.addData("SECOND: ", x + 0.07);
                    telemetry.addData("INCHES: ", xperInch * x);
                }
                else if(tagOfInterest.id == TAG3)
                {
                    telemetry.addLine("TAG3");
                    telemetry.addData("FIRST: ", x + 0.07);
                    telemetry.addData("SECOND: ", xperInch * x);

                }
                else{
                    telemetry.addLine("TAG2");
                    telemetry.addData("FIRST: ", x+ 0.07);
                    telemetry.addData("SECOND: ", xperInch * x);

                }
                telemetry.update();


                TrajectoryActionBuilder trajApril;
                trajApril = drive.actionBuilder(new Pose2d(0, 0, 0))
                        .strafeToLinearHeading(new Vector2d(0, xperInch * x), 0);
                Actions.runBlocking(new SequentialAction(trajApril.build(), new LiftUp()));


                break;
            default:
                throw new Error("Unknown team prop position");
        }

//        Actions.runBlocking(new SequentialAction(
//                trajStart.build(),
//                new DropPixel(),
//                trajBackdrop.build(),
//                //
//                new LiftUp()
//            )
//
//        );

    }

    void tagToTelemetry(org.openftc.apriltag.AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }

}