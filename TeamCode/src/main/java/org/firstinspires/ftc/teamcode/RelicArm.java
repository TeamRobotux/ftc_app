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
    private int extension;


    public RelicArm(HardwareMap hwMap) {
        hand = new TuxServo("hand", hwMap);
        wrist = new TuxServo("wrist", hwMap, false);
        pulley = new TuxMotor("relicPulley", hwMap, 50, 1);
        lazySusan = new TuxMotor("lazySusan", hwMap, 50, 1);
        extension = 0;

    }

    public void rotateCounterClockwise() {
        if (extension < 36) {
            lazySusan.setPower(-.55);
        } else {
            lazySusan.setPower(-.3);
        }
    }

    public void rotateClockwise() {
        if (extension < 36) {
            lazySusan.setPower(.55);
        }
        else {
            lazySusan.setPower(.3);
        }
    }

    public void stopSusan() {
        lazySusan.setPower(0);
    }

    public void stopPulley() {
        pulley.setPower(0);
    }

    public void extend() {
        pulley.setPower(1);
        extension += 1;
    }

    public void retract() {
        pulley.setPower(-1);
        extension -= 1;
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
