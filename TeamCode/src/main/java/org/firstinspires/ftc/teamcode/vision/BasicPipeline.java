package org.firstinspires.ftc.teamcode.vision;

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

// have JunctionPipeline return an x coordinate and a y coordinate in a list, maybe?

public class BasicPipeline extends OpenCvPipeline {

    public Scalar darkestJunctions = new Scalar(80, 100, 100);
    public Scalar lightestJunctions = new Scalar(120, 255, 255);
    public double smallestArea = 4000;
    List<Mat> channels = new ArrayList<>();
    Mat rawHSV = new Mat();
    Mat blurredHSV = new Mat();
    Mat blueMat = new Mat();
    Mat thresholded = new Mat();
    int junctionNumAttr = 0;
    Point junctionPointAttr = new Point();
    double junctionDistanceAttr = 0;
    double propAreaAttr = 0;

    public Mat processFrame(Mat input) {
        // on middle spike mark x = 300. don't think it'll be more than 350
        // on right spike value is 800
        // right between middle and right is 636. possible boundary.
        // TODO: crop the image
        // TODO: log images for debugging


        // crop out parts we're not concerned about

        // Convert image to HSV
        Imgproc.cvtColor(input, rawHSV, Imgproc.COLOR_RGB2HSV);


        // Blur image to lessen noise
        Imgproc.GaussianBlur(rawHSV, blurredHSV, new Size(15, 15), 0); // increase blur?

        Core.inRange(blurredHSV, darkestJunctions, lightestJunctions, thresholded);

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
        }

        Imgproc.drawContours(input, contours, -1, new Scalar(0,255,0), 3);
        Imgproc.drawContours(input, bigContours, -1, new Scalar(255,0,0), 3);

        return input;
    }

    public Point getJunctionPoint() {
        return junctionPointAttr;
    }
    public double getPropAreaAttr() {return propAreaAttr;}
    public int getJunctionNum() {return junctionNumAttr;}
    public double getJunctionDistance() {
        return junctionDistanceAttr; // this is in inches
    }
}