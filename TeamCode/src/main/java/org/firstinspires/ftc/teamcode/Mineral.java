package org.firstinspires.ftc.teamcode;

import org.opencv.core.Point;

import java.util.ArrayList;

/**
 * Created by jack on 10/15/18.
 */

public class Mineral {

    //Mineral sizes

    //Cube side length = 2;
    //max possible cube length = side+ diagonal = 2 + 2.8 = 4.8, somewherer in between?

    //Pixels per inch
    //TODO find actual dpi, get better estimates for gold
    public static final double dpi = 166;
    public static final double goldSize = 3*dpi;
    public static final double silverSize = 2.8*dpi;

    public Point center;
    public double radius;
    public Type type;

    enum Type {GOLD, SILVER}

    public Mineral(Point p, double r, Type t) {
        center = p;
        radius = r;
        type = t;
    }

    public Mineral(Point p, Type t) {
        center = p;
        radius = 0;
        type = t;
    }

    public Mineral() {
        center = new Point(0,0);
        radius = 0;
        type = null;
    }
}
