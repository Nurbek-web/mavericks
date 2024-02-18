package org.firstinspires.ftc.teamcode.common.hardware;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.SampleMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.localizer.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.common.subsystem.DroneSubsystem;
//import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;
//import org.firstinspires.ftc.teamcode.common.subsystem.HangSubsystem;
//import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.util.InverseKinematics;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WServo;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import javax.annotation.concurrent.GuardedBy;

@Config
public class RobotHardware {

    //drivetrain
    public DcMotorEx dtFrontLeftMotor;
    public DcMotorEx dtFrontRightMotor;
    public DcMotorEx dtBackLeftMotor;
    public DcMotorEx dtBackRightMotor;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    // Dead Wheels
    public WEncoder podLeft;
    public WEncoder podRight;
    public WEncoder podFront;

    public DcMotorEx intakeMotor;


    public Servo droneTrigger;

    // hang
    public Servo hangLeftServo;
    public Servo hangRightServo;
    public DcMotorEx hangLeftMotor;
    public DcMotorEx hangRightMotor;

    // Outtake
    public Servo upLeft;
    public Servo upRight;
    public Servo upBack;
    public Servo upFront;
    public Servo downLeft;
    public Servo autoServo;



    public Servo intakeServo;

    public CRServoImplEx intakeRoller;

    // lift motor
    public DcMotorEx liftMotor;

    private HardwareMap hardwareMap;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = 12.0;

    private static RobotHardware instance = null;
    private boolean enabled;

    public List<LynxModule> modules;
    public LynxModule CONTROL_HUB;

    public IMU imu;

    private ArrayList<WSubsystem> subsystems;


    //    public MecanumDrive drivetrain;
    public SampleMecanumDrivetrain drivetrain;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    private Thread imuThread;
    private double imuAngle = 0;
    private double imuOffset = 0;
    private double startOffset = 0;



    public HashMap<Sensors.SensorType, Object> values;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    /**
     * Created at the start of every OpMode.
     *
     * @param hardwareMap The HardwareMap of the robot, storing all hardware devices
     */
    public void init(final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.values = new HashMap<>();

        values.put(Sensors.SensorType.POD_LEFT, 0.0);
        values.put(Sensors.SensorType.POD_FRONT, 0.0);
        values.put(Sensors.SensorType.POD_RIGHT, 0.0);

        // DRIVETRAIN
        this.dtBackLeftMotor = hardwareMap.get(DcMotorEx.class, "dtBackLeftMotor");
        dtBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.dtFrontLeftMotor = hardwareMap.get(DcMotorEx.class, "dtFrontLeftMotor");
        dtFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtFrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.dtBackRightMotor = hardwareMap.get(DcMotorEx.class, "dtBackRightMotor");
        dtBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.dtFrontRightMotor = hardwareMap.get(DcMotorEx.class, "dtFrontRightMotor");
        dtFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        this.liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.hangLeftMotor = hardwareMap.get(DcMotorEx.class, "hangLeftMotor");
        hangLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.hangRightMotor = hardwareMap.get(DcMotorEx.class, "hangRightMotor");
        hangRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.podLeft = new WEncoder(new MotorEx(hardwareMap, "dtFrontRightMotor").encoder);
        this.podFront = new WEncoder(new MotorEx(hardwareMap, "dtBackRightMotor").encoder);
        this.podRight = new WEncoder(new MotorEx(hardwareMap, "dtBackLeftMotor").encoder);

        this.upLeft = hardwareMap.get(Servo.class, "upLeft");
        this.upRight = hardwareMap.get(Servo.class, "upRight");
        this.upBack = hardwareMap.get(Servo.class, "upBack");
        this.upFront = hardwareMap.get(Servo.class, "upFront");
        this.downLeft = hardwareMap.get(Servo.class, "downLeft");
        this.autoServo = hardwareMap.get(Servo.class, "autoServo");
        this.droneTrigger = hardwareMap.get(Servo.class,"droneTrigger");

        autoServo.setPosition(1);
        upRight.setDirection(Servo.Direction.REVERSE);
        upLeft.setDirection(Servo.Direction.FORWARD);


        this.hangLeftServo = hardwareMap.get(Servo.class, "hangLeftServo");
        this.hangRightServo = hardwareMap.get(Servo.class, "hangRightServo");

        this.intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        this.intakeRoller = hardwareMap.get(CRServoImplEx.class, "intakeRoller");

        // Retrieve the IMU from the hardware map
        this.imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        this.imu.initialize(parameters);


        modules = hardwareMap.getAll(LynxModule.class);

        for (LynxModule m : modules) {
            m.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (m.isParent() && LynxConstants.isEmbeddedSerialNumber(m.getSerialNumber()) && CONTROL_HUB == null) CONTROL_HUB = m;
        }

        subsystems = new ArrayList<>();
        drivetrain = new SampleMecanumDrivetrain();
//        extension = new ExtensionSubsystem();
//        intake = new IntakeSubsystem();
        if (Globals.IS_AUTO) {

            startCamera();

//            synchronized (imuLock) {
//                imu = hardwareMap.get(BNO055IMU.class, "imu");
//                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//                parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//                imu.initialize(parameters);
//            }
        } else {
//            drone = new DroneSubsystem();
//            hang = new HangSubsystem();
        }

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void read() {
        // Read all hardware devices here
//        values.put(Sensors.SensorType.EXTENSION_ENCODER, extensionEncoder.getPosition());
//        values.put(Sensors.SensorType.ARM_ENCODER, armPitchEncoder.getCurrentPosition());
        if (Globals.IS_AUTO) {
            values.put(Sensors.SensorType.POD_LEFT, podLeft.getPosition());
            values.put(Sensors.SensorType.POD_FRONT, podFront.getPosition());
            values.put(Sensors.SensorType.POD_RIGHT, podRight.getPosition());
        }
    }

    public void write() {
//        extension.write();
//        intake.write();
//        drivetrain.write();
    }

    public void periodic() {
//        if (voltageTimer.seconds() > 5) {
//            voltageTimer.reset();
//            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
//        }

//        intake.periodic();
//        extension.periodic();
//        drivetrain.periodic();
    }

    public void startIMUThread(LinearOpMode opMode) {
        imuThread = new Thread(() -> {
            while (!opMode.isStopRequested()) {
                synchronized (imuLock) {
                    imuAngle = AngleUnit.normalizeRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + startOffset);
                }
            }
        });
        imuThread.start();
    }

//    public double getAngle() {
//        return AngleUnit.normalizeRadians(imuAngle - imuOffset);
//    }

    public void reset() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.reset();
        }

        imuOffset = imuAngle;
    }

    public double getAngle () {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetIMU() {
        imu.resetYaw();
    }

    public void setStartOffset(double off) {
        startOffset = off;
    }

    public void clearBulkCache() {
        CONTROL_HUB.clearBulkCache();
    }

    public void addSubsystem(WSubsystem... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }


    public void dropPixel() {
        autoServo.setPosition(0.68);
    }

    public double getVoltage() {
        return voltage;
    }

    public double doubleSubscriber(Sensors.SensorType topic) {
        Object value = values.getOrDefault(topic, 0.0);
        if (value instanceof Integer) {
            return ((Integer) value).doubleValue();
        } else if (value instanceof Double) {
            return (Double) value;
        } else {
            throw new ClassCastException();
        }
    }

    public int intSubscriber(Sensors.SensorType topic) {
        Object value = values.getOrDefault(topic, 0);
        if (value instanceof Integer) {
            return (Integer) value;
        } else if (value instanceof Double) {
            return ((Double) value).intValue();
        } else {
            throw new ClassCastException();
        }
    }

    public boolean boolSubscriber(Sensors.SensorType topic) {
        return (boolean) values.getOrDefault(topic, 0);
    }


    public void startCamera() {
        aprilTag = new AprilTagProcessor.Builder()
                // calibrated using 3DF Zephyr 7.021
                .setLensIntrinsics(549.651, 549.651, 317.108, 236.644)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .enableLiveView(false)
                .build();
    }

    public VisionPortal.CameraState getCameraState() {
        if (visionPortal != null) return visionPortal.getCameraState();
        return null;
    }

//    public Pose getAprilTagPosition() {
//        if (aprilTag != null && localizer != null) {
//            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//
//            List<Pose> backdropPositions = new ArrayList<>();
//            for (AprilTagDetection detection : currentDetections) {
//                if (detection.metadata != null) {
//                    switch (detection.id) {
//                        case 1:
//                        case 4:
//                            backdropPositions.add(new Pose(detection.ftcPose).add(new Pose(6, 0, 0)));
//                            break;
//                        case 2:
//                        case 5:
//                            backdropPositions.add(new Pose(detection.ftcPose));
//                            break;
//                        case 3:
//                        case 6:
//                            backdropPositions.add(new Pose(detection.ftcPose).subt(new Pose(6, 0, 0)));
//                            break;
//                        default:
//                            break;
//                    }
//                }
//            }
//
//            Pose backdropPosition = backdropPositions.stream().reduce(Pose::add).orElse(new Pose());
//            backdropPosition = backdropPosition.divide(new Pose(backdropPositions.size(), backdropPositions.size(), backdropPositions.size()));
//
//
//            Pose globalTagPosition = localizer.getPose().x > 0 ?
//                    AprilTagLocalizer.convertBlueBackdropPoseToGlobal(backdropPosition) :
//                    AprilTagLocalizer.convertRedBackdropPoseToGlobal(backdropPosition);
//
//            if (Double.isNaN(globalTagPosition.x) || Double.isNaN(globalTagPosition.y) || Double.isNaN(globalTagPosition.heading)) return null;
//            return globalTagPosition;
//        } else {
//            return null;
//        }
//    }


                            public void closeCamera() {
        if (visionPortal != null) visionPortal.close();
    }

    public void kill() {
        instance = null;
    }
}
