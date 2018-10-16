package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by guinea on 10/5/17.
 * -------------------------------------------------------------------------------------
 * Copyright (c) 2018 FTC Team 5484 Enderbots
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *
 * By downloading, copying, installing or using the software you agree to this license.
 * If you do not agree to this license, do not download, install,
 * copy or use the software.
 * -------------------------------------------------------------------------------------
 * A nice demo class for using OpenCVPipeline. This one also demonstrates how to use OpenCV to threshold
 * for a certain color (blue) and find contours of objects of that color, which is very common in
 * robotics OpenCV applications.
 */

public class MineralDetector  {
    private boolean showContours = true;
    // To keep it such that we don't have to instantiate a new Mat every call to processFrame,
    // we declare the Mats up here and reuse them. This is easier on the garbage collector.
    private Mat hsv = new Mat();
    private Mat whiteThresholded = new Mat();
    private Mat goldThresholded = new Mat();

    enum goldPosition {RIGHT, CENTER, LEFT};

    public goldPosition getPositions(Bitmap input) {
        ArrayList<Mineral> minerals = processFrame(input);

        Mineral circle1 = new Mineral();
        Mineral circle2 = new Mineral();
        Mineral cube = new Mineral();

        boolean cubeFound = false;

        for(Mineral m : minerals) {
            if(!cubeFound && m.type == Mineral.Type.SILVER) {
                circle1 = m;
                cubeFound = true;
            }
            else if(m.type == Mineral.Type.SILVER) {
                circle2 = m;
            }
            else {
                cube = m;
            }
        }

        if(cube.center.x < circle1.center.x && cube.center.x < circle2.center.x) {
            return goldPosition.LEFT;
        }
        else if(cube.center.x > circle1.center.x && cube.center.x > circle2.center.x) {
            return goldPosition.RIGHT;
        }
        else {
            return goldPosition.CENTER;
        }
    }

    public ArrayList<Mineral> processFrame(Bitmap input) {

        Mat rgba = new Mat();
        Utils.bitmapToMat(input, rgba);

        // First, we change the colorspace from RGBA to HSV, which is usually better for color
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);

        //Seperate into H, S, V channels
        ArrayList<Mat> whiteChannels = new ArrayList<Mat>();
        ArrayList<Mat> goldChannels = new ArrayList<Mat>();
        Core.split(hsv, whiteChannels);
        Core.split(hsv, goldChannels);

        //create binary maps of each channel that meets the certain ranges
        Core.inRange(whiteChannels.get(0), new Scalar(89), new Scalar(130), whiteChannels.get(0));
        Core.inRange(whiteChannels.get(1), new Scalar(0), new Scalar(90), whiteChannels.get(1));
        Core.inRange(whiteChannels.get(2), new Scalar(150), new Scalar(255), whiteChannels.get(2));

        Core.inRange(goldChannels.get(0), new Scalar(0), new Scalar(30), goldChannels.get(0));
        Core.inRange(goldChannels.get(1), new Scalar(190), new Scalar(255), goldChannels.get(1));
        Core.inRange(goldChannels.get(2), new Scalar(10), new Scalar(255), goldChannels.get(2));



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
        Imgproc.HoughCircles(whiteBinaryMap, whiteCircles, Imgproc.HOUGH_GRADIENT, .5, 100, 350, 30);

        //Draw the centers and the circles found by HoughCircles onto the base image.
        Mat retImg = new Mat();
        rgba.copyTo(retImg);

        ArrayList<Mineral> minerals = new ArrayList<Mineral>();

        if(!whiteCircles.empty()) {
            for(int i = 0; i < whiteCircles.rows(); i++) {
                int x = (int) Math.round(whiteCircles.get(i, 0)[0]);
                int y = (int) Math.round(whiteCircles.get(i, 0)[1]);
                int r = (int) Math.round(whiteCircles.get(i, 0)[2]);

                minerals.add(new Mineral(new Point(x, y), r, Mineral.Type.SILVER));
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
                if(Imgproc.contourArea(contour) > 5000) {
                    MatOfPoint2f approxPolyDP = new MatOfPoint2f();
                    Imgproc.approxPolyDP(contour, approxPolyDP, .005 * Imgproc.arcLength(contour, true), true);
                    curveContours.add(approxPolyDP);

                    Moments m = Imgproc.moments(contour);
                    int cX = (int) (m.get_m10() / m.get_m00());
                    int cY = (int) (m.get_m01() / m.get_m00());

                    minerals.add(new Mineral(new Point(cX, cY), Math.sqrt(4*Imgproc.contourArea(contour)/Math.PI), Mineral.Type.GOLD));
                }
            }
            ArrayList<MatOfPoint> temp = new ArrayList<MatOfPoint>();

            for(MatOfPoint2f tempC:curveContours) {
                temp.add(new MatOfPoint(tempC.toArray()));
            }
        }

        ArrayList<ArrayList<Point>> retArr = new ArrayList<ArrayList<Point>>();

        return minerals;
    }



}