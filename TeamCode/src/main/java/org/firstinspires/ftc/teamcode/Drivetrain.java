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
        driveFrontR = new TuxMotor("driveFrontR", hwMap, 40*(3/4), 2, 1);
        driveRearR = new TuxMotor("driveRearR", hwMap, 40*(3/4), 2, 1);
        driveFrontL = new TuxMotor("driveFrontL", hwMap, 40*(3/4), 2, -1);
        driveRearL = new TuxMotor("driveRearL", hwMap, 40*(3/4), 2, -1);

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

    public void turn(String direction) {
        int turnCoeff = 1;
        if(direction == "RIGHT") {
            turnCoeff*= -1;
        }

        driveFrontR.setPower(1*turnCoeff);
        driveRearR.setPower(1*turnCoeff);
        driveFrontL.setPower(-1*turnCoeff);
        driveRearL.setPower(-1*turnCoeff);
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
