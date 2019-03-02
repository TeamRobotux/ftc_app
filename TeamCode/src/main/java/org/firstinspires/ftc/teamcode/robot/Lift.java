package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by jack on 10/15/18.
 */

public class Lift implements IsBusy{
    public TuxMotor liftPulley;
    //lift has a 4:1 gear ratio
    //1 revolution of output shaft = 1440 ticks
    //radius of pulley = 1.45669/2 in
    //TODO find diameter using calipers
    //given distance 2, actual distance 6.1
    //2 *tpi/6.1
    //hook to hook = 2.375

    private double tpi = ((((1440*4/(Math.pow(1.45669/2, 2)*Math.PI))*2/6.1*2/5.85)*2/.5)*2/3)*6/5.25*6/5.74*6.5/6.2;

    //the length of the lift in inches

    //should be .75
    private final double liftHeight = 7.85;
    private final int liftHeightTicks = 14052;

    public Lift(HardwareMap hwMap) {
        liftPulley = new TuxMotor("liftPulley", hwMap, tpi, 1120*4, 1);
        liftPulley.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int getEncoderVal() {
        return liftPulley.getEncoderVal();
    }

    public void lowerLift() {
        liftPulley.moveToEncoderVal(0, 1);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        liftPulley.setRunMode(runMode);
    }

    public void logEncoderValues() {
        liftPulley.logEncoderValues();
    }

    public void raiseLift() {
        liftPulley.moveToEncoderVal(-liftHeightTicks,1);
    }

    public boolean isBusy() {
        return liftPulley.isBusy();
    }

    public void setPower(double power) {
        liftPulley.setPower(power);
    }


}
