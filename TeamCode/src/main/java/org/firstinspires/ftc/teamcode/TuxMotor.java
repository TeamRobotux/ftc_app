package org.firstinspires.ftc.teamcode;

/**
 * Created by JackT on 11/4/2017.
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class TuxMotor implements IsBusy{
    private DcMotorEx motor;

    //Number of encoder pulses per revolution
    private double ticksPerInch;

    int reverse;

    //Radius is in inches.
    public TuxMotor(String name, HardwareMap map, double tpi, int reversed) {
        motor = (DcMotorEx) map.get(DcMotor.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ticksPerInch = tpi;
        setPower(0);
        reverse = reversed;

    }

    public TuxMotor(String name, HardwareMap map, long tpi, int reversed, DcMotor.ZeroPowerBehavior behavior) {
        motor = (DcMotorEx) map.get(DcMotor.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ticksPerInch = tpi;
        setPower(0);
        reverse = reversed;
        motor.setZeroPowerBehavior(behavior);
    }

    public void moveDistance(double inches) {
        int ticks = (int) (inches*ticksPerInch + .5);
        moveTicks(ticks);
    }

    public void moveRotation(double rotation) {
        int ticks = (int) (ticksPerInch*rotation);
        moveTicks(ticks);
    }


    public void moveTicks(int ticks) {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(ticks*reverse);

        motor.setPower(.3);
    }

    public int getEncoderVal() {
        return motor.getCurrentPosition();
    }

    public int getTolerance() { return motor.getTargetPositionTolerance(); };

    public void setTolerance(int tolerance) { motor.setTargetPositionTolerance(tolerance);}

    public void setPower(double power) {
        if(motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        motor.setPower(power*reverse);
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    public String getPIDCoefficients(DcMotor.RunMode r) {
        String returnString = ""+ motor.getPIDFCoefficients(r).p;
        returnString += " " + motor.getPIDFCoefficients(r).i;
        returnString += " " + motor.getPIDFCoefficients(r).d;
        return returnString;
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

}
