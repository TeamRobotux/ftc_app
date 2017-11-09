package org.firstinspires.ftc.teamcode;

/**
 * Created by JackT on 11/4/2017.
 */

import com.qualcomm.robotcore.hardware.HardwareMap;



public class Drivetrain {
    private TuxMotor  driveFrontR  = null;
    private TuxMotor  driveRearR  = null;
    private TuxMotor  driveFrontL  = null;
    private TuxMotor  driveRearL  = null;

    private TuxMotor[] wheels;

    public Drivetrain(HardwareMap hwMap) {

        //Wheel motors; Gear Ratio of sprockets = 3:4, motor gearratio 40:1, 2 inch radii
        driveFrontR = new TuxMotor("driveFrontR", hwMap, 1, -1);
        driveRearR = new TuxMotor("driveRearR", hwMap, 1, -1);
        driveFrontL = new TuxMotor("driveFrontL", hwMap, 1, 1);
        driveRearL = new TuxMotor("driveRearL", hwMap, 1, 1);

        wheels = new TuxMotor[4];

        wheels[0] = driveRearL;
        wheels[1] = driveRearR;
        wheels[2] = driveFrontR;
        wheels[3] = driveFrontL;

    }

    public void drivePower(float power) {
        for(TuxMotor m : wheels) {
            m.setPower(power);
        }
    }

    public void driveSinglePower(float power, int motor) {
        wheels[motor].setPower(power);
    }

    public void driveTicks(int ticks) {
        for(TuxMotor m : wheels) {
            m.moveTicks(ticks);
        }
    }

    public void turn(float power) {
        driveFrontR.setPower(power);
        driveRearR.setPower(power);
        driveFrontL.setPower(-power);
        driveRearL.setPower(-power);
    }

    public void strafe(float power) {
        driveFrontR.setPower(-power);
        driveRearR.setPower(power);
        driveFrontL.setPower(power);
        driveRearL.setPower(-power);

    }

    //move a distance (in inches)
    public void driveDistance(double distance)  {
        for(TuxMotor m : wheels) {
            m.moveDistance(distance);
        }
    }

    public boolean isBusy() {
        boolean busy = false;
        for(TuxMotor m : wheels) {
            busy = busy || m.isBusy();
        }
        return busy;
    }


}
