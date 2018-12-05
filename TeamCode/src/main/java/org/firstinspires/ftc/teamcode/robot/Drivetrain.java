package org.firstinspires.ftc.teamcode.robot;


import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by JackT on 11/4/2017.
 */





public class Drivetrain implements IsBusy{
    private TuxMotor driveFrontR  = null;
    private TuxMotor  driveRearR  = null;
    private TuxMotor  driveFrontL  = null;
    private TuxMotor  driveRearL  = null;

    private TuxMotor[] wheels;

    private PIDController[] wheelControllers = new PIDController[4];



    double strafeTpi = 63.661977237*24/(16.25)*24/25.25*24/24.5*24/23.75;

    public Drivetrain(HardwareMap hwMap) {

        double tpi = 63.661977237;
        // 1120 pulses per 1 axle rotation (1120 tpr)
        // 4 shaft turns per 3 wheel turns (Gear Ratio of sprockets = 4:3)
        // 1 wheel turn per 4*pi inches
        // 118.835690842 pulses per inch

        double tpr = 1120*4/3;

        driveFrontR = new TuxMotor("driveFrontR", hwMap, tpi, tpr, -1);
        driveRearR = new TuxMotor("driveRearR", hwMap, tpi, tpr, -1);
        driveFrontL = new TuxMotor("driveFrontL", hwMap, tpi, tpr, 1);
        driveRearL = new TuxMotor("driveRearL", hwMap, tpi, tpr, 1);



        wheels = new TuxMotor[4];

        wheels[0] = driveRearL;
        wheels[1] = driveRearR;
        wheels[2] = driveFrontR;
        wheels[3] = driveFrontL;


        double p = 1/100;
        double i = 0;
        double d =0;
        int t = 30;

        for(int j = 0; j < 4; j++) {
            wheelControllers[j] = new PIDController(p, i, d, t);
        }

        for(TuxMotor m: wheels) {
            m.setTolerance(30);
        }

    }

    public void setRunMode(DcMotor.RunMode runMode) {
        for(TuxMotor m : wheels) {
            m.setRunMode(runMode);
        }
    }

    public void drivePower(double power) {
        for(TuxMotor m : wheels) {
            m.setPower(power*0.8);
        }
    }


    //x = strafe y = drive
    public void drive360(double x, double y, double turn) {
        driveRearR.setPower(y - x - 2*turn);
        driveFrontR.setPower(y + x - 2*turn);
        driveRearL.setPower(y + x + 2*turn);
        driveFrontL.setPower(y - x + 2*turn);
    }


    public void strafeDiagonal(double x, double y) {
        if((x > 0 && y > 0) || (x < 0 && y > 0)) {
            driveFrontL.setPower(x);
            driveRearR.setPower(x);
        }
        else {
            driveFrontR.setPower(x);
            driveRearL.setPower(x);
        }

    }

    public void driveSinglePower(double power, int motor) {
        wheels[motor].setPower(power);
    }

    public void driveTicks(int ticks) {
        for(TuxMotor m : wheels) {
            m.moveTicks(ticks);
        }
    }

    public void turn(double power) {
        driveFrontR.setPower(-power);
        driveRearR.setPower(-power);
        driveFrontL.setPower(power);
        driveRearL.setPower(power);
    }

    public void strafe(double powerInit) {
        double power = powerInit*.8;
        driveFrontR.setPower(-power);
        driveRearR.setPower(power);
        driveFrontL.setPower(power);
        driveRearL.setPower(-power);

    }

    public void resetEncoders() {
        for(TuxMotor m : wheels) {
            m.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void strafeDistance(double inches) {
        int ticks = (int) (inches*strafeTpi + .5);
        driveFrontR.moveTicks(-ticks);
        driveRearR.moveTicks(ticks);
        driveFrontL.moveTicks(ticks);
        driveRearL.moveTicks(-ticks);
    }

    public void strafeDistance(double inches, double power) {
        int ticks = (int) (inches*strafeTpi + .5);
        driveFrontR.moveTicks(-ticks, power);
        driveRearR.moveTicks(ticks, power);
        driveFrontL.moveTicks(ticks, power);
        driveRearL.moveTicks(-ticks, power);
    }

    //move a distance (in inches)
    public void driveDistance(double distance)  {
        for(TuxMotor m : wheels) {
            m.moveDistance(distance);
        }
    }

    public void driveDistance(double distance, double power) {
        for(TuxMotor m : wheels) {
            m.moveDistance(distance, power);
        }
    }

    public boolean isBusy() {
    boolean busy = false;
    for(TuxMotor m : wheels) {
        busy = busy || m.isBusy();
    }
    return busy;
    }

    public void driveDistanceCustom(double distance) {
        for(int i = 0; i < 4; i++) {
            wheelControllers[i].setGoal(wheels[i].getTicksfromDistance(distance), wheels[i].getEncoderVal());
        }

        while(!checkAllMotorsArrived()) {
            for(int i = 0; i < 4; i++) {
                wheels[i].setPower(wheelControllers[i].getOutput(wheels[i].getEncoderVal()));
            }
        }
    }

    public void strafeDistanceCustom(double distance) {
        for(int i = 0; i < 4; i++) {
            wheelControllers[i].setGoal((int) distance*strafeTpi + .5, wheels[i].getEncoderVal());
        }

        while(!checkAllMotorsArrived()) {
            for(int i = 0; i < 4; i++) {

                double initialPower = wheelControllers[i].getOutput(wheels[i].getEncoderVal());

                if(i == 0 || i == 1) {
                    initialPower = -initialPower;
                }

                wheels[i].setPower(initialPower);
            }
        }
    }

    public void strafeDistanceCustom(double distance, double expectedHeading, double currentAngle) {
        for(int i = 0; i < 4; i++) {
            wheelControllers[i].setGoal(wheels[i].getTicksfromDistance(distance), wheels[i].getEncoderVal());
        }

        while(!checkAllMotorsArrived()) {
            for(int i = 0; i < 4; i++) {
                double initialPower = wheelControllers[i].getOutput(wheels[i].getEncoderVal());
                double error = (expectedHeading-currentAngle)/expectedHeading;

                if(i == 0 || i == 3) {
                    initialPower = -initialPower;
                }

                if(i < 2) {
                    error = -error;
                }

                if(Math.abs(error) > 3) {
                    wheels[i].setPower(initialPower + error);
                }
            }
        }
    }

    public void driveDistanceCustom(double distance, double expectedHeading, double currentAngle) {
        for(int i = 0; i < 4; i++) {
            wheelControllers[i].setGoal((int) distance*strafeTpi + .5, wheels[i].getEncoderVal());
        }

        while(!checkAllMotorsArrived()) {
            for(int i = 0; i < 4; i++) {
                double initialPower = wheelControllers[i].getOutput(wheels[i].getEncoderVal());
                double error = (expectedHeading-currentAngle)/expectedHeading;

                if(i < 2) {
                    error = -error;
                }

                if(Math.abs(error) > 3) {
                    wheels[i].setPower(initialPower + error);
                }
            }
        }
    }

    public void logEncoderValues() {
        String logString = "";
        for(TuxMotor m : wheels) {
            logString += m.getEncoderVal() + ", ";
        }

        Log.i("drivetrain", logString);
    }

    public boolean checkAllMotorsArrived() {
        for(PIDController p : wheelControllers) {
            if(p.getError() > p.getTolerance()) {
                return false;
            }
        }
        return true;
    }
}
