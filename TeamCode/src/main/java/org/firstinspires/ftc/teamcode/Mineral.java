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

    public static final double mineralNearbyDistance = 5;

    public Point center;
    public double radius;
    public Type type;
    public int count;

    enum Type {GOLD, SILVER}

    public Mineral(Point p, double r, Type t) {
        center = p;
        radius = r;
        type = t;
        count = 1;
    }

    public Mineral(Point p, Type t) {
        center = p;
        radius = 0;
        type = t;
        count = 1;
    }

    public Mineral(Point p, double r, Type t, int c) {
        center = p;
        radius = 0;
        type = t;
        count = c;
    }

    public Mineral() {
        center = new Point(0,0);
        radius = 0;
        type = null;
    }

    public void averageLocation(Mineral m) {
        center.x = (center.x*count + m.center.x)/(count+1);
        center.y = (center.y*count + m.center.y)/(count+1);
        count++;
    }

    public boolean mineralIsNear(Mineral m) {
        return Math.abs(m.center.x - center.x) < mineralNearbyDistance && Math.abs(m.center.y-center.y) < mineralNearbyDistance;
    }
}
