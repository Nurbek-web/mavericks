//package org.firstinspires.ftc.teamcode.common.vision;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//public class NewAprilTag extends OpMode {
//    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
//    AprilTagProcessor myAprilTagProcessor;
//    VisionPortal myVisionPortal;
//
//
//    @Override
//    public void init() {
//        myAprilTagProcessor = new AprilTagProcessor.Builder()
//                .setDrawTagID(true)
//                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
//                .setDrawTagOutline(true)
//                .setDrawAxes(true)
//                .setDrawCubeProjection(true)
//                .build();
//        myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), myAprilTagProcessor);
//
//        myVisionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(myAprilTagProcessor)
//                .setCameraResolution(new Size(640, 480))
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .enableCameraMonitoring(true)
//                .setAutoStopLiveView(true)
//                .build();
//
//    }
//    @Override
//    public void loop() {
//
//    }
//}
