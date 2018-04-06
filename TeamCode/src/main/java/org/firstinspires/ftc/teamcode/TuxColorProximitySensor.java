package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by jack on 3/22/18.
 */

public class TuxColorProximitySensor {
    public DistanceSensor distanceSensor;
    public ColorSensor colorSensor;

    public TuxColorProximitySensor(HardwareMap hwMap) {
        distanceSensor = hwMap.get(DistanceSensor.class, "jewelColorDistance");
        colorSensor = hwMap.get(ColorSensor.class, "jewelColorDistance");
    }

    public int getBlue() {
        return colorSensor.blue();
    }

    public int getRed() {
        return colorSensor.red();
    }

    public int getGreen() {
        return colorSensor.green();
    }

    public int getAlpha() {
        return colorSensor.alpha();
    }

    public int getHue() {
        return colorSensor.argb();
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }
}
