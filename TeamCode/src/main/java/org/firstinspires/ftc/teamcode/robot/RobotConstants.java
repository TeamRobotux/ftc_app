package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

/**
 * Created by jack on 11/4/18.
 */

@Config
public class RobotConstants {
    public static int goldHLow = 0;
    public static int goldHHigh = (30);
    public static int goldSLow = (160);
    public static int goldSHigh = (220);
    public static int goldVLow = (90);
    public static int goldVHigh = (255);

    public static int silverHLow = 0;
    public static int silverHHigh = 200  ;
    public static int silverSLow = (0);
    public static int silverSHigh = (10);
    public static int silverVLow = (240);
    public static int silverVHigh = (255);

    public static boolean gold = false;

    public static int channel = 2;

    public static boolean finalMap = false;

    public static double HoughDp = .005;
    public static double HoughDist = 100;
    public static int Hough1 = 400;
    public static int Hough2 = 20;
}
