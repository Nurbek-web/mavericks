package org.firstinspires.ftc.teamcode.opmode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
@Autonomous(name = "BlueFarAuto \uD83D\uDD35", group = "Autonomous")
public class BlueFarAuto extends LinearOpMode {
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

            AprilTagDetectionPipeline aprilTagDetectionPipeline;
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

            org.openftc.apriltag.AprilTagDetection tagOfInterest = null;

            camera.setPipeline(aprilTagDetectionPipeline);

            ArrayList<AprilTagDetection> currentDetections;
            runtime.reset();
            while (!tagFound && runtime.seconds() <= 5.0)
            {
                currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                for(org.openftc.apriltag.AprilTagDetection tag : currentDetections)
                {
                    if((loc==Location.LEFT && (tag.id==1 || tag.id==4))
                            || (loc==Location.CENTER && (tag.id==2 || tag.id==5))
                            || (loc==Location.RIGHT && (tag.id==3 || tag.id==6)))
                    {
                        tagFound = true;
                        tagOfInterest = tag;
                        break;
                    }
                }
            }
            if (tagFound) {
                drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

                double x = tagOfInterest.pose.x - 0.095;
                double xperInch = 40;

                TrajectoryActionBuilder trajApril;
                if(tagOfInterest.id==3 || tagOfInterest.id==6){
                    trajApril = drive.actionBuilder(new Pose2d(0, 0, 0))
                            .strafeToConstantHeading(new Vector2d(0, xperInch * x+2));
                    Actions.runBlocking(new SequentialAction(
                            trajApril.build()
                    ));
                }else{

                    trajApril = drive.actionBuilder(new Pose2d(0, 0, 0))
                            .strafeToConstantHeading(new Vector2d(0, xperInch * x));
                    Actions.runBlocking(new SequentialAction(
                            trajApril.build()
                    ));
                }
            }
            Actions.runBlocking(new SequentialAction(
                    new LiftUp()
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

            if (pos < 800) {
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
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webka"));
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

        if (pipeline.getJunctionPoint().x < 330) {
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
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-35, 60, Math.toRadians(90)));
        lift = new LiftSubsystem();

        autoServo = hardwareMap.get(Servo.class, "autoServo");
        robot.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        autoServo.setPosition(1);
        lift.closeOuttake();
        lift.intendOuttake();

        waitForStart();

        telemetry.addLine("STARTED");
        blueFar(drive, loc);

    }

    private void blueFar(MecanumDrive drive, Location loc) {

        TrajectoryActionBuilder trajStart, trajBackdrop;

        Vector2d startingPosition;
        switch(loc){
            case RIGHT: // right
                trajStart = drive
                        .actionBuilder(new Pose2d(-35, 60, Math.toRadians(90)))

                        .strafeToConstantHeading(new Vector2d(-35, 37))
                        .turn(-Math.toRadians(90));
//                        .strafeToConstantHeading(new Vector2d(5, 37))
//                        .strafeToConstantHeading(new Vector2d(12, 37));
                trajBackdrop = drive.actionBuilder(new Pose2d(-36, 37, 0))
                        .strafeToConstantHeading(new Vector2d(-35, 58))
                        .turn(Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(20, 57))
                        .splineToSplineHeading(new Pose2d(37.3, 34, Math.PI), 0);
                break;
            case CENTER: // center
                trajStart = drive
                        .actionBuilder(new Pose2d(-35, 60, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(-35, 30))
                        .strafeToConstantHeading(new Vector2d(-35, 36));
                trajBackdrop = drive.actionBuilder(new Pose2d(-35, 36, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(-35, 58))
                        .strafeToConstantHeading(new Vector2d(20, 57))
                        .splineToSplineHeading(new Pose2d(37.3, 40, Math.PI), 0);

                break;
            case LEFT: // left
                trajStart = drive
                        .actionBuilder(new Pose2d(-35, 58, Math.toRadians(90)))
                        .strafeToConstantHeading(new Vector2d(-36, 34))
                        .turn(Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(-32, 34))
                        .strafeToConstantHeading(new Vector2d(-34, 34));
//                        .strafeToConstantHeading(new Vector2d(18, 34))
//                        .strafeToConstantHeading(new Vector2d(14, 34));
                trajBackdrop = drive.actionBuilder(new Pose2d(-34, 34, Math.toRadians(180)))
                        .strafeToConstantHeading(new Vector2d(-36, 34))
                        .strafeToConstantHeading(new Vector2d(-35, 58))
                        .turn(-Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(20, 57))
                        .strafeToLinearHeading(new Vector2d(37.3, 47.5), Math.toRadians(180));
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
    }



}