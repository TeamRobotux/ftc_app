package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by jack on 9/26/18.
 */

public class RobotHardware {

    //Testing stuff
    public CRServo intakeL = null;
    public CRServo intakeR = null;

    /* Constructor */
    public RobotHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        //Testing stuff
        intakeL = ahwMap.get(CRServo.class, "intakeL");
        intakeR = ahwMap.get(CRServo.class, "intakeR");
    }


}
