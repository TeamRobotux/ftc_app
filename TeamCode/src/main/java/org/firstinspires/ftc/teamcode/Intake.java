package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImpl;


/**
 * Created by JackT on 11/9/2017.
 */

public class Intake {
    private TuxServo rightRot;
    private TuxCRServo rightWheel;
    private TuxServo leftRot;
    private TuxCRServo leftWheel;

//    private final double rOpen = .9;
//    private final double rClosed = 1;
//    private final double lOpen =.9;
//    private final double lClosed = 1;


    public Intake(HardwareMap hwMap) {
        rightRot = new TuxServo("intakeR", hwMap);
        rightRot.setDirection(Servo.Direction.REVERSE);
        rightRot.moveTo(0);

        rightWheel = new TuxCRServo("intakeWheelR", hwMap, CRServo.Direction.FORWARD);

        leftRot = new TuxServo("intakeL", hwMap);

        leftWheel = new TuxCRServo("intakeWheelL", hwMap, CRServo.Direction.REVERSE);

        stopRot();
    }

    public void open() {
        rightRot.moveTo(.85);
        leftRot.moveTo(.85);
    }

    public void close() {
        rightRot.moveTo(1);
        leftRot.moveTo(1);
    }

    public void rotateIn() {
        rightWheel.move(-1);
        leftWheel.move(-1);
    }

    public void rotateOut() {
        rightWheel.move(1);
        leftWheel.move(1);
    }

    public void reset() {
        rightRot.moveTo(0);
        leftRot.moveTo(0);
    }

    public void movePerp() {
        rightRot.moveTo(.3);
        leftRot.moveTo(.3);
    }

    public void setLocation(double power) {
        rightRot.moveTo(power);
        leftRot.moveTo(power);
    }

    public void stopRot() {
        rightWheel.stop();
        leftWheel.stop();

    }
}
