package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Created by JackT on 11/9/2017.
 */

public class GlyphGrabber {
    private TuxServo right;
    private TuxServo left;

    public GlyphGrabber(HardwareMap hwMap) {
        right = new TuxServo("glyphR", hwMap);
        left = new TuxServo("glyphL", hwMap);


        left.setDirection(Servo.Direction.REVERSE);

        left.moveTo(0.0);
        right.moveTo(0.0);
    }

    public void open() {
        right.move(TuxServo.Dir.FORWARD);
        left.move(TuxServo.Dir.FORWARD);
    }

    public void close() {
        right.move(TuxServo.Dir.BACK);
        left.move(TuxServo.Dir.BACK);
    }

    public void stop() {
        right.stop();
        left.stop();
    }

    @Override
    public String toString() {
        double rightPos = right.getPosition();
        double leftPos = left.getPosition();

        if (rightPos != leftPos) {
            return "WARNING: UNEQUAL VALUES! LEFT == " + leftPos + " RIGHT == " + rightPos;
        }
        return "" + rightPos;
    }
}
