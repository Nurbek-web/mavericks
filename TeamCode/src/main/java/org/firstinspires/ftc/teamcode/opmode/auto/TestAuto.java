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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.common.vision.Location;
import org.firstinspires.ftc.teamcode.vision.BasicPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


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

            if (pos < 1200) {
                // true causes the action to rerun
                return true;
            } else {
                // false stops action rerun
                robot.liftMotor.setPower(0);
                lift.extend1Outtake();
                sleep(3000);
                lift.openOuttake();
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

        if (pipeline.getJunctionPoint().x < 170) {
            telemetry.addLine("LEFT PROP");
            loc = Location.LEFT;
        }
        else if (pipeline.getJunctionPoint().x > 750) {
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

        waitForStart();

        telemetry.addLine("STARTED");
        blueFar(drive, loc);

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
                telemetry.addLine("LEFT");
                trajStart = drive.actionBuilder(new Pose2d(-34, 60, Math.toRadians(90)))
                        .lineToY(46).strafeToLinearHeading(new Vector2d(-34, 30), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(-27.5, 30), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(-34, 30), Math.toRadians(180));
                trajBackdrop = drive.actionBuilder(new Pose2d(
                                -34, 30, Math.toRadians(180)))
                        .strafeToConstantHeading(new Vector2d(-34, 58))
                        .strafeToConstantHeading(new Vector2d(12, 58))
                        .splineToConstantHeading(new Vector2d(39.1, 41), Math.PI / 2);
                break;
            default:
                throw new Error("Unknown team prop position");
        }

        Actions.runBlocking(new SequentialAction(
                trajStart.build(),
                new DropPixel(),
                trajBackdrop.build(),
                new LiftUp()
            )

        );

    }

}