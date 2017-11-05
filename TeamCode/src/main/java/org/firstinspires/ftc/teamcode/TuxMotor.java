package org.firstinspires.ftc.teamcode;

/**
 * Created by JackT on 11/4/2017.
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TuxMotor{
    private DcMotor motor;
    //Number of encoder pulses per revolution
    private long ppr;
    //length of radius in inches
    private double circumfrence;

    int reverse;

    //Radius is in inches.
    public TuxMotor(String name, HardwareMap map, int gearRatio, double r, int reversed) {
        motor = map.get(DcMotor.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        circumfrence = r*2*Math.PI;
        ppr = 7*gearRatio;
        setPower(0);
        reverse = reversed;
    }

    public void moveDistance(double inches) {
        double revolutions = inches/circumfrence;

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setTargetPosition((int) Math.round(revolutions*ppr*reverse));

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(.4);
    }

    public void moveTicks(int ticks) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setTargetPosition(ticks*reverse);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(.4);
    }

    public int getEncoderVal() {
        return motor.getCurrentPosition();
    }

    public void setPower(float power) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power*reverse);
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

}
000