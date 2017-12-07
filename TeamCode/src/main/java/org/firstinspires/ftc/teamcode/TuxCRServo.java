package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Created by JackT on 11/9/2017.
 */

public class TuxCRServo {
    private CRServo servo;

    //int reverse;

    public TuxCRServo(String name, HardwareMap hwmap, CRServo.Direction dir) {
        servo = hwmap.get(CRServo.class, name);
        servo.setDirection(dir);
        stop();

        //reverse = reversed;
    }


    public void setDirection(CRServo.Direction d) {servo.setDirection(d);}


    public void move(double power) {
        servo.setPower(power);
    }

    public void stop() {
        servo.setPower(0);
    }


}
