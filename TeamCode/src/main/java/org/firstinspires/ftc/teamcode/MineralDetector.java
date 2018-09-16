package org.firstinspires.ftc.teamcode;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

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

public class MineralDetector extends OpenCVPipeline {
    private boolean showContours = true;
    // To keep it such that we don't have to instantiate a new Mat every call to processFrame,
    // we declare the Mats up here and reuse them. This is easier on the garbage collector.
    private Mat hsv = new Mat();
    private Mat whiteThresholded = new Mat();
    private Mat goldThresholded = new Mat();

    // This is called every camera frame.
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
        Core.inRange(whiteChannels.get(0), new Scalar(0), new Scalar(180), whiteChannels.get(0));
        Core.inRange(whiteChannels.get(1), new Scalar(0), new Scalar(20), whiteChannels.get(1));
        Core.inRange(whiteChannels.get(2), new Scalar(0), new Scalar(255), whiteChannels.get(2));

        Core.inRange(goldChannels.get(0), new Scalar(10), new Scalar(30), whiteChannels.get(0));
        Core.inRange(goldChannels.get(1), new Scalar(90), new Scalar(255), whiteChannels.get(1));
        Core.inRange(goldChannels.get(2), new Scalar(90), new Scalar(255), whiteChannels.get(2));

        //Recombine all the layers, finding all the pixels which fall into each range
        Mat whiteBinaryMap = new Mat();
        Core.bitwise_and(whiteChannels.get(0), whiteChannels.get(1), whiteBinaryMap);
        Core.bitwise_and(whiteChannels.get(2), whiteBinaryMap, whiteBinaryMap);

        Mat goldBinaryMap = new Mat();
        Core.bitwise_and(goldChannels.get(0), goldChannels.get(1), goldBinaryMap);
        Core.bitwise_and(goldChannels.get(2), goldBinaryMap, goldBinaryMap);

        // we blur the binaryImage image to remove noise (using Gaussian Blur)
        Imgproc.GaussianBlur(whiteBinaryMap, whiteBinaryMap, new Size(9,9), 2, 2);
        Imgproc.GaussianBlur(goldBinaryMap, goldBinaryMap, new Size(9,9), 2, 2);

        //Circle Detection

        //Use the Hough Transform to detect circles. The circles mat is fillled with the locationsn and radius of each circle.
        Mat whiteCircles = new Mat();
        Imgproc.HoughCircles(whiteBinaryMap, whiteCircles, Imgproc.HOUGH_GRADIENT, 1.2, 100, 350, 30);

        //Draw the centers and the circles found by HoughCircles onto the base image.
        Mat retImg = new Mat();
        rgba.copyTo(retImg);


        if(!whiteCircles.empty()) {
            for(int i = 0; i < whiteCircles.rows(); i++) {
                int x = (int) Math.round(whiteCircles.get(i, 0)[0]);
                int y = (int) Math.round(whiteCircles.get(i, 0)[1]);
                int r = (int) Math.round(whiteCircles.get(i, 0)[2]);
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
                if(Imgproc.contourArea(contour) > 5000) {
                    MatOfPoint2f approxPolyDP = new MatOfPoint2f();
                    Imgproc.approxPolyDP(contour, approxPolyDP, .06 * Imgproc.arcLength(contour, true), true);
                    curveContours.add(approxPolyDP);
                }
            }
            ArrayList<MatOfPoint> temp = new ArrayList<MatOfPoint>();

            for(MatOfPoint2f tempC:curveContours) {
                temp.add(new MatOfPoint(tempC.toArray()));
            }
            Imgproc.drawContours(retImg, temp, -1, new Scalar(255, 0, 0), 4);
        }

        return retImg; // display the image seen by the camera
    }
}