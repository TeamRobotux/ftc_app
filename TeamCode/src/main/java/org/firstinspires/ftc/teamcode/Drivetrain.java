package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by JackT on 11/4/2017.
 */





public class Drivetrain implements IsBusy{
    private TuxMotor  driveFrontR  = null;
    private TuxMotor  driveRearR  = null;
    private TuxMotor  driveFrontL  = null;
    private TuxMotor  driveRearL  = null;

    private TuxMotor[] wheels;

    public Drivetrain(HardwareMap hwMap) {

        double tpi = 118.835690842;
        // 1120 pulses per 1 axle rotation (1120 tpr)
        // 4 shaft turns per 3 wheel turns (Gear Ratio of sprockets = 4:3)
        // 1 wheel turn per 4*pi inches
        // 118.835690842 pulses per inch

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
        driveFrontR.setPower(y - 2*turn);
        driveRearL.setPower(y + x + 2*turn);
        driveFrontL.setPower(y + 2*turn);
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
        driveFrontR.moveDistance(-inches * Math.tan(Math.PI / 180 * 50));
        driveRearR.moveDistance(inches * Math.tan(Math.PI / 180 * 50));
        driveFrontL.moveDistance(inches * Math.tan(Math.PI / 180 * 50));
        driveRearL.moveDistance(-inches * Math.tan(Math.PI / 180 * 50));
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
