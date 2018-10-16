package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;

/**
 * Created by jack on 9/26/18.
 */

public class RobotHardware implements IsBusy{

    //Testing stuff
    public CRServo intakeL = null;
    public CRServo intakeR = null;
    public Lift lift= null;
    public Drivetrain drivetrain = null;
    public WebcamName webcamName = null;
    public TuxGyro gyro = null;


    //TODO find actual values
    public final double cameraAngle = 15;
    public final Point cameraCenter = new Point(310, 230);
    public final double cameraOffset = 20;


    /* Constructor */
    public RobotHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        //Testing stuff
        intakeL = ahwMap.get(CRServo.class, "intakeL");
        intakeR = ahwMap.get(CRServo.class, "intakeR");
        lift = new Lift(ahwMap);
        drivetrain = new Drivetrain(ahwMap);
        gyro = new TuxGyro(ahwMap);

        webcamName = ahwMap.get(WebcamName.class, "webcam");
    }

    @Override
    public boolean isBusy() {
        return lift.isBusy() || drivetrain.isBusy();
    }
}
