package org.firstinspires.ftc.teamcode.common.vision;

import org.opencv.core.*;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// have JunctionPipeline return an x coordinate and a y coordinate in a list, maybe?

public class RedPipeline extends OpenCvPipeline {
    Telemetry telemetry;

    public RedPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }


    public Scalar darkestJunctions = new Scalar(0, 167, 85);
    public Scalar lightestJunctions = new Scalar(255, 255, 128);
    public double smallestArea = 4000;
    List<Mat> channels = new ArrayList<>();
    Mat rawYCrCb = new Mat();
    Mat blurred = new Mat();
    Mat blueMat = new Mat();
    Mat thresholded = new Mat();
    int junctionNumAttr = 0;
    Point junctionPointAttr = new Point();
    double junctionDistanceAttr = 0;
    double propAreaAttr;

    public Mat processFrame(Mat input) {

        // crop out parts we're not concerned about

        // Convert image to HSV
        Imgproc.cvtColor(input, rawYCrCb, Imgproc.COLOR_RGB2YCrCb);

        // Blur image to lessen noise
        Imgproc.GaussianBlur(rawYCrCb, blurred, new Size(15, 15), 0); // increase blur?

        Core.inRange(blurred, darkestJunctions, lightestJunctions, thresholded);

        /*
        Core.split(input, channels);
        blueMat = channels.get(0);

        // Threshold image, turning it into binary (only black and white). Now openCV knows what to get the contour, or shape, of.
        Core.inRange(blueMat, darkestJunctions, lightestJunctions, thresholded);
        */


        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Get distance and centroid of biggest junction
        List<MatOfPoint> bigContours = new ArrayList<>();
        if (!contours.isEmpty()) {
            MatOfPoint biggestContour = contours.get(0);

            for (MatOfPoint curContour : contours) {
                if (Imgproc.contourArea(curContour) > Imgproc.contourArea(biggestContour)) {
                    biggestContour = curContour;
                }
            }
            bigContours.add(biggestContour);

            // Find centroid
            Moments moments = Imgproc.moments(biggestContour);
            Point junctionPoint = new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00());

            //Assign attributes
            junctionNumAttr = contours.size();
            propAreaAttr = Imgproc.contourArea(biggestContour);
            junctionDistanceAttr = 240000/Imgproc.contourArea(biggestContour);
            junctionPointAttr = junctionPoint;

            telemetry.addLine("Place the purple pixel between the second and third compliant wheels from the left.");
            telemetry.addLine("It should be roughly centered.  It should be as close to touching the ground as possible WITHOUT touching the ground.");
            telemetry.addLine("Ensure the intake is at the bottom of its backlash-induced free-spinning zone so the pixel doesn't scrape the ground.");
            telemetry.addLine("The pan should be FULLY ON THE GROUND when the program starts.");
            telemetry.addData("Prop x value: ", getJunctionPoint().x);
            telemetry.addData("Prop area: ", getPropAreaAttr());

            telemetry.update();
        }

        Imgproc.drawContours(input, contours, -1, new Scalar(0,255,0), 3);
        Imgproc.drawContours(input, bigContours, -1, new Scalar(255,0,0), 3);

        return input;
    }

    public Point getJunctionPoint() {
        return junctionPointAttr;
    }

    public int getJunctionNum() {return junctionNumAttr;}
    public double getPropAreaAttr() {return propAreaAttr;}
    public double getJunctionDistance() {
        return junctionDistanceAttr; // this is in inches
    }
}
