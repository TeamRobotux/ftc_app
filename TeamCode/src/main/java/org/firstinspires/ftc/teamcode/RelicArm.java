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


    public RelicArm(HardwareMap hwMap) {
        hand = new TuxServo("hand", hwMap);
        wrist = new TuxServo("wrist", hwMap);
        pulley = new TuxMotor("relicPulley", hwMap, 50, 1);
        lazySusan = new TuxMotor("lazySusan", hwMap, 50, 1);

    }

    public void rotateCounterClockwise() {
        lazySusan.setPower(-.55);
    }

    public void rotateClockwise() {
        lazySusan.setPower(.55);
    }

    public void stopSusan() {
        lazySusan.setPower(0);
    }

    public void stopPulley() {
        pulley.setPower(0);
    }

    public void extend() {
        pulley.setPower(-1);
    }

    public void retract() {
        pulley.setPower(1);
    }

    public void closeHand() {
        hand.moveTo(0);
    }

    public void openHand() {
        hand.moveTo(1);
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
