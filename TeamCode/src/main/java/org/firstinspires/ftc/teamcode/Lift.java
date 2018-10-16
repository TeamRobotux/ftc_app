package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by jack on 10/15/18.
 */

public class Lift implements IsBusy{
    public TuxMotor liftPulley;
    //TODO find actual tpi
    private final double tpi = 20;

    //the length of the lift in inches
    //TODO find actual lift length
    private final double liftLength = 10;

    public Lift(HardwareMap hwMap) {
        liftPulley = new TuxMotor("liftPulley", hwMap, tpi,1);
        liftPulley.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void lowerLift() {
        liftPulley.moveDistance(-liftLength);
    }

    public void raiseLift() {
        liftPulley.moveDistance(liftLength);
    }

    public boolean isBusy() {
        return liftPulley.isBusy();
    }


}
