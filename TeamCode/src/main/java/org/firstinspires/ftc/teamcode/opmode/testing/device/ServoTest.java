package org.firstinspires.ftc.teamcode.opmode.testing.device;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.util.wrappers.WServo;

@Config
@TeleOp(name = "ServoTest")
public class ServoTest extends OpMode {
    public Servo autoServo;

    public static double target = 0.0;

    @Override
    public void init() { // hold Drone - 0.57; launch drone - 0.8
        this.autoServo = hardwareMap.get(Servo.class, "droneTrigger");
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            autoServo.setPosition(target);
        }
    }
}
