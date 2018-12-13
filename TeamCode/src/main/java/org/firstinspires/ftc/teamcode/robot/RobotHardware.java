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

    //Testing stuff
    public Lift lift= null;
    public Drivetrain drivetrain = null;
    public TuxGyro gyro = null;
    public Intake intake = null;

    public TuxMotor scoopMotor = null;

    public TuxSwitch sideSwitch = null;

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

        sideSwitch = new TuxSwitch("sideSwitch", ahwMap);
    }


    @Override
    public boolean isBusy() {
        return lift.isBusy() || drivetrain.isBusy() || intake.isBusy() || scoopMotor.isBusy();
    }


}
