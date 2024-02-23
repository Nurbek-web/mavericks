package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.vision.RedPipeline;
import org.firstinspires.ftc.teamcode.vision.sim.BasicPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "PropTesting")
public class TestVision extends OpMode {
//    PropPipeline pipeline = new PropPipeline();
    RedPipeline pipeline = new RedPipeline();
    OpenCvCamera camera;
    FtcDashboard dashboard = FtcDashboard.getInstance();



    // Get apriltag ID from prop situation
    double propX = pipeline.getJunctionPoint().x;
    double propArea = pipeline.getPropAreaAttr();

    RobotHardware robot;

    @Override
    public void init() {
        robot = RobotHardware.getInstance();
//        // Camera activation
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
        FtcDashboard.getInstance().startCameraStream(camera, 0);

            telemetry.addLine("Place the purple pixel between the second and third compliant wheels from the left.");
            telemetry.addLine("It should be roughly centered.  It should be as close to touching the ground as possible WITHOUT touching the ground.");
            telemetry.addLine("Ensure the intake is at the bottom of its backlash-induced free-spinning zone so the pixel doesn't scrape the ground.");
            telemetry.addLine("The pan should be FULLY ON THE GROUND when the program starts.");
            telemetry.addData("Prop x value: ", pipeline.getJunctionPoint().x);
            telemetry.addData("Prop area: ", pipeline.getPropAreaAttr());
//        if (purpleHook.getPosition() != Constants.purpleHookDragPosition) purpleHook.setPosition(Constants.purpleHookDragPosition);
//        if (autoHook.getPosition() != Constants.autoHookStowPosition) {
//            autoHook.setPosition(Constants.autoHookStowPosition);
//        }
            if (pipeline.getJunctionPoint().x < 170) {
                telemetry.addLine("LEFT PROP");
            }
            else if (pipeline.getJunctionPoint().x > 750) {
                telemetry.addLine("RIGHT PROP");
            } else {
                telemetry.addLine("CENTER PROP");
            }
//
//
            telemetry.update();
//        }
            // LEFT: x value= 86
            // CENTER: x value = 582
            // RIGHT x value = 1200

    }

    @Override
    public void loop() {
        telemetry.addLine("Place the purple pixel between the second and third compliant wheels from the left.");
        telemetry.addLine("It should be roughly centered.  It should be as close to touching the ground as possible WITHOUT touching the ground.");
        telemetry.addLine("Ensure the intake is at the bottom of its backlash-induced free-spinning zone so the pixel doesn't scrape the ground.");
        telemetry.addLine("The pan should be FULLY ON THE GROUND when the program starts.");
        telemetry.addData("Prop x value: ", pipeline.getJunctionPoint().x);
        telemetry.addData("Prop area: ", pipeline.getPropAreaAttr());
        telemetry.update();
    }
}
