package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by jack on 10/15/18.
 */

public class Lift implements IsBusy{
    public TuxMotor liftPulley;
    //lift has a 4:1 gear ratio
    //1 revolution of output shaft = 1120 ticks
    //radius of pulley = 1.45669 in
    //TODO find diameter using calipers
    private final double tpi = 420*4/(Math.pow(1.45669, 2)*Math.PI);

    //the length of the lift in inches

    //should be .75
    private final double liftHeightUp = 2;
    private final double liftHeightDown = 10.25;

    public Lift(HardwareMap hwMap) {
        liftPulley = new TuxMotor("liftPulley", hwMap, tpi, 1120*4, 1);
        liftPulley.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void lowerLift() {
        liftPulley.moveDistance(-liftHeightUp);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        liftPulley.setRunMode(runMode);
    }

    public void logEncoderValues() {
        liftPulley.logEncoderValues();
    }

    public void raiseLift() {
        liftPulley.moveDistance(liftHeightUp);
    }

    public boolean isBusy() {
        return liftPulley.isBusy();
    }

    public void setPower(double power) {
        liftPulley.setPower(power);
    }


}
