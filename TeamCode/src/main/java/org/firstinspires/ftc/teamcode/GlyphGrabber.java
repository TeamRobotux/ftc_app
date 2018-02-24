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
    private TuxCRServo rightT;
    private TuxCRServo leftT;

    private TuxCRServo flipper;

    private boolean flipped;

    public GlyphGrabber(HardwareMap hwMap) {
        rightD = new TuxCRServo("glyphRD", hwMap, CRServo.Direction.REVERSE);
        leftD = new TuxCRServo("glyphLD", hwMap, CRServo.Direction.FORWARD);
        rightT = new TuxCRServo("glyphRT", hwMap, CRServo.Direction.FORWARD);
        leftT = new TuxCRServo("glyphLT", hwMap, CRServo.Direction.REVERSE);

        flipper = new TuxCRServo("flipper", hwMap, CRServo.Direction.FORWARD);

        flipped = false;
    }

    public void rotClockwise() {
        flipper.move(-1);
        flipped = false;
    }

    public void rotCounterClockwise() {
        flipper.move(1);
        flipped = true;
    }

    public void stopRot() {
        flipper.move(0);
    }

    public void suck() {
        if(flipped) {
            rightT.move(1);
            leftT.move(1);
        }
        else {
            rightD.move(1);
            leftD.move(1);
        }
    }

    public void push() {
        if(flipped) {
            rightT.move(-1);
            leftT.move(-1);
        }
        else {
            rightD.move(-1);
            leftD.move(-1);
        }
    }

    public void pushAll() {
        rightD.move(-1);
        leftD.move(-1);
        rightT.move(-1);
        leftT.move(-1);
    }

    public void stop() {
        rightD.stop();
        leftD.stop();
        rightT.stop();
        leftT.stop();
    }
}
