package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Created by JackT on 11/9/2017.
 */

public class TuxServo {
    private Servo servo;

    //int reverse;

    public TuxServo(String name, HardwareMap hwmap) {
        servo = hwmap.get(Servo.class, name);
        servo.setPosition(0);

        //reverse = reversed;
    }

    enum Dir {FORWARD,BACK}

    public void setDirection(Servo.Direction d) {servo.setDirection(d);}


    public void move(Dir d) {
        if(d == Dir.FORWARD) {
            servo.setPosition(servo.getPosition()+.05);
        }
        else {
            servo.setPosition(servo.getPosition()-.05);
        }
    }

    public void moveTo(double pos) {
        servo.setPosition(pos);
    }

    public void stop() { servo.setPosition(servo.getPosition());}

    public double getPosition() {
        return servo.getPosition();
    }

    public void setScale(double scaleMin, double scaleMax) {
        servo.scaleRange(scaleMin, scaleMax);
    }
}
