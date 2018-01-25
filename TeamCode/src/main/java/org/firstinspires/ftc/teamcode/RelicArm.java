package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Created by JackT on 11/9/2017.
 */

public class RelicArm {
    private TuxServo hand;
    private TuxServo wrist;
    private TuxMotor pulley;
    private TuxMotor lazySusan;
    private double handPos;
    private final double handOpen = .45;
    private final double handClosed = .1;
    private final double wristOpen =.52d;
    private final double wristClosed = .1;


    public RelicArm(HardwareMap hwMap) {
        hand = new TuxServo("hand", hwMap);
        wrist = new TuxServo("wrist", hwMap);
        pulley = new TuxMotor("relicPulley", hwMap, 50, 1);
        lazySusan = new TuxMotor("lazySusan", hwMap, 50, 1);

        hand.moveTo(0);
        wrist.moveTo(0);


    }

    public void rotateCounterClockwise() {
        lazySusan.setPower(1);
    }

    public void rotateClockwise() {
        lazySusan.setPower(-1);
    }

    public void stopSusan() {
        lazySusan.setPower(0);
    }

    public void stopPulley() {
        pulley.setPower(0);
    }

    public void extend() {
        pulley.setPower(1);
    }

    public void retract() {
        pulley.setPower(-1);
    }

    public void closeHand() {
        if(handPos > 0) {
            handPos -= .10;
        }
        hand.moveTo(handPos);
    }

    public void openHand() {
        if(handPos < 1) {
            handPos += .1;
        }
        hand.moveTo(handPos);
    }

    public void lowerWrist() {
        wrist.moveTo(0);
    }

    public void raiseWrist() {
        wrist.moveTo(1);
    }

    public void moveWristParallel() {
        wrist.moveTo(.66);
    }



}
