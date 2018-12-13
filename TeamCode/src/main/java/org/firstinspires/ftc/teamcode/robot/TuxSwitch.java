package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by jack on 12/11/18.
 */

public class TuxSwitch {
    DigitalChannel sw;

    public TuxSwitch(String name, HardwareMap hwMap) {
        sw = hwMap.get(DigitalChannel.class, name);

        sw.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getState() {
        return sw.getState();
    }
}
