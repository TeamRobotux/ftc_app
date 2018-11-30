package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by jack on 11/30/18.
 */

public class Intake implements IsBusy{

    public TuxMotor grabber;
    public TuxMotor pulley;

    public TuxServo lifter;
    public TuxServo gate;

    //TODO find all these

    private double openGatePos = 0;
    private double closedGatePos = .5;

    private double perpIntakePos = .5;
    private double raisedIntakePos = 1;
    private double loweredIntakePos = .2;

    public final double extensionThreshold = 1000;

    private double scoringPos = 1500;

    public Intake(HardwareMap hwMap) {
        //tpi and tpr using counts of Neverrest 20 output shaft
        grabber = new TuxMotor("intakeGrabber", hwMap, 538, 538, -1, DcMotor.ZeroPowerBehavior.FLOAT);

        //tpr = output shaft counts of neverrest 40, tpi = tpr/ circumference = 1120/(2*pi*1.45669) = 122
        pulley = new TuxMotor("intakePulley", hwMap, 1120/(2*Math.PI*1.45669), 1120, 1);

        lifter = new TuxServo("intakeLifter", hwMap, true);
        gate = new TuxServo("intakeGate", hwMap, true);
    }

    public void moveIntakePerp() {
        lifter.moveTo(perpIntakePos);
    }

    public void lowerIntake() {
        lifter.moveTo(loweredIntakePos);
    }

    public void raiseIntake() {
        lifter.moveTo(raisedIntakePos);
    }

    public void closeGate() {
        gate.moveTo(closedGatePos);
    }

    public void openGate() {
        gate.moveTo(openGatePos);
    }

    public int getPulleyPosition() {
        return pulley.getEncoderVal();
    }

    public void intake() {
        grabber.setPower(1);
    }

    public void outtake() {
        grabber.setPower(-1);
    }

    public void stopIntaking() {
        grabber.setPower(0);
    }

    public void extend() {
        pulley.setPower(1);
    }

    public void retract() {
        pulley.setPower(-1);
    }

    public void stopExtension() {
        pulley.setPower(0);
    }



    @Override
    public boolean isBusy() {
        return pulley.isBusy();
    }
}
