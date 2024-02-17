//package org.firstinspires.ftc.teamcode.vision;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//import java.util.Arrays;
//
//public class BarcodePipeline extends OpenCvPipeline {
//    public enum Position {
//        Reading,
//        One,
//        Two,
//        Three
//    }
//
//    public Position position = Position.Reading;
//
//
//    Mat hue = new Mat();
//    Mat saturation = new Mat();
//    Mat value = new Mat();
//    ArrayList<Mat> channels = new ArrayList<Mat>();
//    Mat redtmp1 = new Mat();
//    Mat redtmp2 = new Mat();
//
//    Mat hsv = new Mat();
//    Mat cropped = new Mat();
//    Mat thresholded = new Mat();
//
//    Mat leftThird = new Mat();
//    Mat middleThird = new Mat();
//    Mat rightThird = new Mat();
//
//    Mat outthing = new Mat();
//
//    int leftNumber = 0;
//    int middleNumber = 0;
//    int rightNumber = 0;
//    int swapNumber = 0;
//
//    int output = 0;
//
//    @Override
//    public void init(Mat input) {
//        channels.add(hue);
//        channels.add(saturation);
//        channels.add(value);
//    }
//
//    @Override
//    public Mat processFrame(Mat input) {
//        // Imgproc
//
//        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
//
//        cropped = hsv.submat(input.rows() / 4, input.rows() - (input.rows() / 4), 24, input.cols() - 24);
//
//        Core.split(cropped, channels);
//
//        if (side == RobotSide.Red || side == RobotSide.JankTwo) { // Invert this after testing is complete
//            Core.inRange(cropped, new Scalar(0, 70, 100), new Scalar(10, 255, 255), redtmp1);
//            Core.inRange(cropped, new Scalar(170, 70, 100), new Scalar(180, 255, 255), redtmp2);
//
//            Core.bitwise_or(redtmp1, redtmp2, thresholded);
//        } else {
//            //Core.inRange(cropped, new Scalar(186, 60, 67), new Scalar(), thresholded);
//            Core.inRange(cropped, new Scalar(100, 150, 100), new Scalar(120, 245, 255), thresholded);
//        }
//
//        leftThird = thresholded.colRange(0, thresholded.cols() / 3);
//        middleThird = thresholded.colRange(thresholded.cols() / 3, 2 * (thresholded.cols() / 3));
//        rightThird = thresholded.colRange(2 * (thresholded.cols() / 3), thresholded.cols());
//
//        leftNumber = Core.countNonZero(leftThird);
//        middleNumber = Core.countNonZero(middleThird);
//        rightNumber = Core.countNonZero(rightThird);
//
//        if (side == RobotSide.Red) {
//            swapNumber = rightNumber;
//            rightNumber = leftNumber;
//            leftNumber = swapNumber;
//        }
//
//        if (leftNumber < middleNumber && leftNumber < rightNumber) position = Position.One;
//        if (middleNumber < leftNumber && middleNumber < rightNumber) position = Position.Two;
//        if (rightNumber < leftNumber && rightNumber < middleNumber) position = Position.Three;
//
//        Imgproc.putText(thresholded, String.format("Thirds: %d %d %d", leftNumber, middleNumber, rightNumber), new Point(0, 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255));
//        Imgproc.putText(thresholded, String.format("Determination: %s", position.toString()), new Point(0, 50), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 255, 255));
//
//        return thresholded;
//    }
//}