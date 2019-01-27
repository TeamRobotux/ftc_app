package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;

/**
 * Created by jack on 9/26/18.
 */

public class RobotHardware implements IsBusy {

    public double markerVert = .62;
    public double markerScore = .14;

    //Testing stuff
    public Lift lift= null;
    public Drivetrain drivetrain = null;
    public TuxGyro gyro = null;
    public Intake intake = null;

    public TuxMotor scoopMotor = null;
    public TuxServo scoopServo = null;

    public TuxSwitch sideSwitch = null;

    public TuxServo markerServo = null;

    /* Constructor */
    public RobotHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        //Testing stuff
        lift = new Lift(ahwMap);
        drivetrain = new Drivetrain(ahwMap, RobotConstants.PIDP, RobotConstants.PIDI, RobotConstants.PIDD, RobotConstants.PIDT);
        gyro = new TuxGyro(ahwMap);

        intake = new Intake(ahwMap);
        //output shaft ticks = 1120
        //no gear ratio
        scoopMotor = new TuxMotor("scoopMotor", ahwMap, 0, 1120,1, DcMotor.ZeroPowerBehavior.BRAKE);
        scoopServo = new TuxServo("scoopServo", ahwMap, true);

        sideSwitch = new TuxSwitch("sideSwitch", ahwMap);

        markerServo = new TuxServo("markerServo", ahwMap, false);
        markerServo.moveTo(markerVert);
    }


    @Override
    public boolean isBusy() {
        return lift.isBusy() || drivetrain.isBusy() || intake.isBusy() || scoopMotor.isBusy();
    }


}
