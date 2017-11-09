package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Created by JackT on 11/9/2017.
 */

public class GlyphGrabber {
    private TuxServo right;
    private TuxServo left;
    private final double rOpen = .34;
    private final double rClosed = .2;
    private final double lOpen = .55;
    private final double lClosed = .3;


    public GlyphGrabber(HardwareMap hwMap) {
        right = new TuxServo("glyphR", hwMap);
        left = new TuxServo("glyphL", hwMap);

        //Right open == .35, closed == .25
        //Left open = .55, closed ==.4, .35

        left.setDirection(Servo.Direction.REVERSE);

        left.moveTo(lClosed);
        right.moveTo(rClosed);
    }

    public void open() {
        right.moveTo(rOpen);
        left.moveTo(lOpen);
    }

    public void close() {
        right.moveTo(rClosed);
        left.moveTo(lClosed);
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
