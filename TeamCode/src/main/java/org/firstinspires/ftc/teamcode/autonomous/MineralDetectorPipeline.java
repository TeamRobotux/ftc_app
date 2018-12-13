package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;

/**
 * Created by jack on 11/3/18.
 */

public class MineralDetectorPipeline extends OpenCVPipeline {

    private boolean showContours = true;
    // To keep it such that we don't have to instantiate a new Mat every call to processFrame,
    // we declare the Mats up here and reuse them. This is easier on the garbage collector.
    private Mat hsv = new Mat();
    private Mat whiteThresholded = new Mat();
    private Mat goldThresholded = new Mat();

    private Mineral silverMineral1 = new Mineral();
    private Mineral silverMineral2 = new Mineral();
    private Mineral goldMineral = new Mineral();

    private Mineral firstMineral = new Mineral();
    private Mineral lowestMineral = new Mineral();

    private ArrayList<Mineral> minerals = new ArrayList<Mineral>();

    enum goldPosition {RIGHT, CENTER, LEFT}

    public MineralDetector.goldPosition getPositions(TelemetryPacket t) {

        if(minerals.size() == 0) { return MineralDetector.goldPosition.CENTER; }

        for(Mineral m : minerals) {
            if(m.type == Mineral.Type.SILVER) {
                if(m.count > silverMineral1.count) {
                    silverMineral1 = m;
                }
                else if(m.count > silverMineral2.count) {
                    silverMineral2 = m;
                }
            }
            else {
                if(m.count > goldMineral.count) {
                    goldMineral = m;
                }
            }
        }

        double s1Pos = silverMineral1.center.x;
        double s2Pos = silverMineral2.center.x;
        double goldPos = goldMineral.center.x;



        t.put("Spos1", s1Pos);
        t.put("Spos2", s2Pos);
        t.put("goldPos", goldPos);

        if(goldPos < s1Pos&& goldPos < s2Pos) {
            return MineralDetector.goldPosition.LEFT;
        }
        else if(goldPos > s1Pos && goldPos > s2Pos) {
            return MineralDetector.goldPosition.RIGHT;
        }
        else {
            return MineralDetector.goldPosition.CENTER;
        }
    }

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {

        // First, we change the colorspace from RGBA to HSV, which is usually better for color
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);

        //Seperate into H, S, V channels
        ArrayList<Mat> whiteChannels = new ArrayList<Mat>();
        ArrayList<Mat> goldChannels = new ArrayList<Mat>();
        Core.split(hsv, whiteChannels);
        Core.split(hsv, goldChannels);

        //create binary maps of each channel that meets the certain ranges
        Core.inRange(whiteChannels.get(0), new Scalar(RobotConstants.silverHLow), new Scalar(RobotConstants.silverHHigh), whiteChannels.get(0));
        Core.inRange(whiteChannels.get(1), new Scalar(RobotConstants.silverSLow), new Scalar(RobotConstants.silverSHigh), whiteChannels.get(1));
        Core.inRange(whiteChannels.get(2), new Scalar(RobotConstants.silverVLow), new Scalar(RobotConstants.silverVHigh), whiteChannels.get(2));

        Core.inRange(goldChannels.get(0), new Scalar(RobotConstants.goldHLow), new Scalar(RobotConstants.goldHHigh), goldChannels.get(0));
        Core.inRange(goldChannels.get(1), new Scalar(RobotConstants.goldSLow), new Scalar(RobotConstants.goldSHigh), goldChannels.get(1));
        Core.inRange(goldChannels.get(2), new Scalar(RobotConstants.goldVLow), new Scalar(RobotConstants.goldVHigh), goldChannels.get(2));



        //Recombine all the layers, finding all the pixels which fall into each range
        Mat whiteBinaryMap = new Mat();
        Core.bitwise_and(whiteChannels.get(0), whiteChannels.get(1), whiteBinaryMap);
        Core.bitwise_and(whiteChannels.get(2), whiteBinaryMap, whiteBinaryMap);

        Mat goldBinaryMap = new Mat();
        Core.bitwise_and(goldChannels.get(0), goldChannels.get(1), goldBinaryMap);
        Core.bitwise_and(goldChannels.get(2), goldBinaryMap, goldBinaryMap);

        /*
        //floodfill the map
        Mat whiteFloodFillMap = new Mat();
        Core.copyMakeBorder(whiteBinaryMap, whiteFloodFillMap, 1,1, 1, 1,Core.BORDER_REPLICATE);
        Imgproc.floodFill(whiteBinaryMap, whiteFloodFillMap, new Point(1,1), new Scalar(255, 255, 255));
        Core.bitwise_not(whiteFloodFillMap, whiteFloodFillMap);
        Core.bitwise_or(whiteFloodFillMap, whiteBinaryMap, whiteBinaryMap);

        Mat goldFloodFillMap = new Mat();
        goldBinaryMap.copyTo(goldFloodFillMap);
        Core.copyMakeBorder(goldBinaryMap, goldFloodFillMap, 1,1, 1, 1,Core.BORDER_REPLICATE);
        Imgproc.floodFill(goldBinaryMap, goldFloodFillMap, new Point(1,1), new Scalar(255, 255, 255));
        Core.bitwise_not(goldFloodFillMap, goldFloodFillMap);
        Core.bitwise_or(goldFloodFillMap, goldBinaryMap, goldBinaryMap);
        */

        // we blur the binaryImage image to remove noise (using Gaussian Blur)
        Imgproc.GaussianBlur(whiteBinaryMap, whiteBinaryMap, new Size(9,9), 2, 2);
        Imgproc.GaussianBlur(goldBinaryMap, goldBinaryMap, new Size(9,9), 2, 2);

        //Circle Detection

        //Use the Hough Transform to detect circles. The circles mat is filled with the locations and radius of each circle.
        Mat whiteCircles = new Mat();
        Imgproc.HoughCircles(whiteBinaryMap, whiteCircles, Imgproc.HOUGH_GRADIENT, RobotConstants.HoughDp, RobotConstants.HoughDist, RobotConstants.Hough1, RobotConstants.Hough2);

        //Draw the centers and the circles found by HoughCircles onto the base image.
        Mat retImg = new Mat();
        rgba.copyTo(retImg);

        ArrayList<Mineral> newMinerals = new ArrayList<Mineral>();

        if(!whiteCircles.empty()) {
            for(int i = 0; i < whiteCircles.rows(); i++) {
                int x = (int) Math.round(whiteCircles.get(i, 0)[0]);
                int y = (int) Math.round(whiteCircles.get(i, 0)[1]);
                int r = (int) Math.round(whiteCircles.get(i, 0)[2]);

                newMinerals.add(new Mineral(new Point(x, y), r, Mineral.Type.SILVER));
                Imgproc.circle(retImg, new Point(x, y), r, new Scalar(0, 255, 255), 4);
                Imgproc.rectangle(retImg, new Point(x - 5, y - 5), new Point(x + 5, y + 5), new Scalar(0, 128, 255), -1);
            }
        }

        //Cube Detection
        //Find contours in the source image
        ArrayList<MatOfPoint> goldMapContours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(goldBinaryMap, goldMapContours, hierarchy, Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE);


        ArrayList<MatOfPoint2f> goldMapContours2f = new ArrayList<MatOfPoint2f>();
        //convert goldMap Contours to matofpoint2f
        for(MatOfPoint mat:goldMapContours) {
            goldMapContours2f.add(new MatOfPoint2f(mat.toArray()));
        }

        if(!goldMapContours.isEmpty()) {
            ArrayList<MatOfPoint2f> curveContours = new ArrayList<MatOfPoint2f>();
            for(MatOfPoint2f contour:goldMapContours2f) {
                if(Imgproc.contourArea(contour) > 50) {
                    MatOfPoint2f approxPolyDP = new MatOfPoint2f();
                    Imgproc.approxPolyDP(contour, approxPolyDP, .005 * Imgproc.arcLength(contour, true), true);
                    curveContours.add(approxPolyDP);

                    Moments m = Imgproc.moments(contour);
                    int cX = (int) (m.get_m10() / m.get_m00());
                    int cY = (int) (m.get_m01() / m.get_m00());

                    newMinerals.add(new Mineral(new Point(cX, cY), Math.sqrt(4*Imgproc.contourArea(contour)/Math.PI), Mineral.Type.GOLD));
                    Imgproc.circle(retImg, new Point(cX, cY), 7, new Scalar(255, 255, 255), -1);
                }
            }
            ArrayList<MatOfPoint> temp = new ArrayList<MatOfPoint>();

            for(MatOfPoint2f tempC:curveContours) {
                temp.add(new MatOfPoint(tempC.toArray()));
            }
            Imgproc.drawContours(retImg, temp, -1, new Scalar(255, 0, 0), 4);
        }

        if(newMinerals.size() > 0) {
            averagePosition(newMinerals);
        }

        if(newMinerals.size() > 0) {
            ArrayList<Mineral> sortedMinerals = sortMinerals(newMinerals);
            lowestMineral = sortedMinerals.get(sortedMinerals.size()-1);
        }

        return retImg;
    }

    public void averagePosition(ArrayList<Mineral> inputMinerals) {
        for(Mineral newMineral : inputMinerals) {
            boolean matched = false;
            for(Mineral oldMineral : minerals) {
                if(newMineral.type == oldMineral.type && oldMineral.mineralIsNear(newMineral)) {
                    oldMineral.averageLocation(newMineral);
                    matched = true;
                    break;
                }
            }
            if(!matched) {
                minerals.add(newMineral);
            }
        }
    }

    public Mineral getFirstMineral() {
        return firstMineral;
    }

    private ArrayList<Mineral> sortMinerals(ArrayList<Mineral> m) {
        int n = m.size();
        for (int i=1; i<n; ++i)
        {
            Mineral key = m.get(i);
            int j = i-1;

            /* Move elements of arr[0..i-1], that are
               greater than key, to one position ahead
               of their current position */
            while (j>=0 && m.get(j).center.y > key.center.y)
            {
                m.set(j+1, m.get(j));
                j = j-1;
            }
            m.set(j+1, key);
        }
        return m;
    }

    public Mineral getLowestMineral() {
        return lowestMineral;
    }



}
