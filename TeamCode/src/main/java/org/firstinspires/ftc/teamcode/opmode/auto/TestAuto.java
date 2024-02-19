package org.firstinspires.ftc.teamcode.opmode.auto;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Autonomous(name = "TestAuto", group = "Autonomous")
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));

        // vision here that outputs position
        int visionOutputPosition = 1;
        Pose2d beginPose = new Pose2d(0, 0, 0);

        Action trajectoryAction1;


        waitForStart();


        Actions.runBlocking(
                drive.actionBuilder(beginPose).splineTo(new Vector2d(0, 0), Math.toRadians(90)).build()
        );
    }
}