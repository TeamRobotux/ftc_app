package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Created by JackT on 11/9/2017.
 */

public class GlyphGrabber {
    private TuxCRServo rightD;
    private TuxCRServo leftD;

    public GlyphGrabber(HardwareMap hwMap) {
        rightD = new TuxCRServo("glyphRD", hwMap, CRServo.Direction.REVERSE);
        leftD = new TuxCRServo("glyphLD", hwMap, CRServo.Direction.FORWARD);
    }

    public void suck() {
        rightD.move(1);
        leftD.move(1);
    }

    public void push() {
        rightD.move(-1);
        leftD.move(-1);
    }

    public void stop() {
        rightD.stop();
        leftD.stop();
    }
}
