package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "LiftTest")
public class LiftTest extends OpMode {
    public DcMotorEx liftMotor;

    private GamepadEx gamepadEx;
    private GamepadEx gamepadEx2;

    public static int TARGET = 0;
    public static double POWER = 0.5;
    private final int lFirstLevelPos = 1400;
    private final int lSecondLevelPos = 700;
    private final int lDownPos = 0;
    private static double upRightPos = 0;
    private static double upBackPos = 0;
    private static double upFrontPos = 0;
    private static double downLeftPos = 0;


    public Servo upRight;
    public Servo holdPixel;
    public Servo upFront;
    public Servo downLeft;
    public Servo upLeft;

    public static double LOW_POS = 0;
    public static double HIGH_POS = 1;

    @Override
    public void init() {
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
//        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        liftMotor.setTargetPosition(lDownPos);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Turn the motor back on when we are done

        this.upLeft = hardwareMap.get(Servo.class, "upLeft");
        this.upRight = hardwareMap.get(Servo.class, "upRight");
        this.holdPixel = hardwareMap.get(Servo.class, "upBack");
        this.upFront = hardwareMap.get(Servo.class, "upFront");
        this.downLeft = hardwareMap.get(Servo.class, "downLeft");

        upRight.setDirection(Servo.Direction.REVERSE);
        upLeft.setDirection(Servo.Direction.FORWARD);

        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void loop() {
        int liftPos = liftMotor.getCurrentPosition();

        liftMotor.setTargetPosition(TARGET);
        liftMotor.setPower(POWER);

        if (gamepad1.a) {
            upRight.setPosition(HIGH_POS);
        }
        if (gamepad1.b) {
            upLeft.setPosition(HIGH_POS);
        }
        if (gamepad1.y) {
            downLeft.setPosition(HIGH_POS);
        }
        if (gamepad1.right_bumper) {
            upFront.setPosition(HIGH_POS);
        }
        if (gamepad1.left_bumper) {
            holdPixel.setPosition(LOW_POS);
        }

        // Get the current position of the armMotor
        double position = liftMotor.getCurrentPosition();

        // Get the target position of the armMotor
        double desiredPosition = liftMotor.getTargetPosition();

        // Show the position of the armMotor on telemetry
        telemetry.addData("Encoder Position", position);

        telemetry.addData("power", POWER);

        // Show the target position of the armMotor on telemetry
        telemetry.addData("Desired Position", desiredPosition);

        telemetry.update();
    }
}
