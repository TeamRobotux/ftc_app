package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by JackT on 11/4/2017.
 */





public class Drivetrain {
    private TuxMotor  driveFrontR  = null;
    private TuxMotor  driveRearR  = null;
    private TuxMotor  driveFrontL  = null;
    private TuxMotor  driveRearL  = null;

    private TuxMotor[] wheels;

    public Drivetrain(HardwareMap hwMap) {

        double tpi = 67.11409395973154;
        // 7 pulses per 1 revolution (7 tpr)
        // 40 revolutions per 1 shaft turn (motor gearratio 40:1)
        // 4 shaft turns per 3 wheel turns (Gear Ratio of sprockets = 3:4)
        // 1 wheel turn per 12.8 inches
        // 29.2 pulses per inch????????

        driveFrontR = new TuxMotor("driveFrontR", hwMap, tpi, -1);
        driveRearR = new TuxMotor("driveRearR", hwMap, tpi, -1);
        driveFrontL = new TuxMotor("driveFrontL", hwMap, tpi, 1);
        driveRearL = new TuxMotor("driveRearL", hwMap, tpi, 1);



        wheels = new TuxMotor[4];

        wheels[0] = driveRearL;
        wheels[1] = driveRearR;
        wheels[2] = driveFrontR;
        wheels[3] = driveFrontL;

        for(TuxMotor m: wheels) {
            m.setTolerance(30);
        }

    }

    public void drivePower(float power) {
        for(TuxMotor m : wheels) {
            m.setPower(power*0.8);
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

    public void strafeDistance(double inches) {
        driveFrontR.moveDistance(-inches);
        driveRearR.moveDistance(inches);
        driveFrontL.moveDistance(inches);
        driveRearL.moveDistance(-inches);
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
