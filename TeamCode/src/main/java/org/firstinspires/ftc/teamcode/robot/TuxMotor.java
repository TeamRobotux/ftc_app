package org.firstinspires.ftc.teamcode.robot;

/**
 * Created by JackT on 11/4/2017.
 */

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.IsBusy;

public class TuxMotor implements IsBusy {
    private DcMotorEx motor;

    //Number of encoder pulses per revolution
    private double ticksPerInch;
    private double ticksPerRevolution;
    private int reverse;

    //Radius is in inches.
    public TuxMotor(String name, HardwareMap map, double tpi, double tpr, int reversed) {
        motor = (DcMotorEx) map.get(DcMotor.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ticksPerInch = tpi;
        ticksPerRevolution = tpr;
        setPower(0);
        reverse = reversed;

    }

    public TuxMotor(String name, HardwareMap map, long tpi, double tpr, int reversed, DcMotor.ZeroPowerBehavior behavior) {
        motor = (DcMotorEx) map.get(DcMotor.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ticksPerInch = tpi;
        ticksPerRevolution = tpr;
        setPower(0);
        reverse = reversed;
        motor.setZeroPowerBehavior(behavior);
    }



    public void moveDistance(double inches) {
        int ticks = (int) (inches*ticksPerInch + .5);
        moveTicks(ticks);
    }

    public void moveDistance(double inches, double power) {
        int ticks = (int) (inches*ticksPerInch + .5);
        moveTicks(ticks, power);
    }


    public void moveRotation(double rotation) {
        int ticks = (int) Math.round(ticksPerRevolution*rotation);
        Log.i("encoderTarget", Integer.valueOf(ticks).toString());
        moveTicks(ticks);
    }

    public void moveRotation(double rotation, double power) {
        int ticks = (int) Math.round(ticksPerRevolution*rotation);
        moveTicks(ticks);
    }

    public void moveDegrees(double degrees) {
        moveRotation(degrees/360);
    }


    public void moveTicks(int ticks) {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(ticks*reverse);
        motor.setPower(.3);
    }

    public void moveTicks(int ticks, double power) {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(ticks*reverse);
        motor.setPower(power);
    }

    public int getEncoderVal() {
        return motor.getCurrentPosition();
    }

    public int getTolerance() { return motor.getTargetPositionTolerance(); };

    public void logEncoderValues() {
        while (motor.isBusy()) {
            Log.i("TuxMotor", Integer.valueOf(getEncoderVal()).toString());
        }
    }

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

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

}
