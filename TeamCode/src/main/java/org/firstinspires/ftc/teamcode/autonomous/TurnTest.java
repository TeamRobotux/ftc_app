package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousBasic;


/**
 * Created by jack on 1/25/19.
 */

@Autonomous(name = "turnTest", group = "Linear Opmode")
public class TurnTest extends AutonomousBasic {

    @Override
    void runAutonomous() {

        turnDegrees(robot, this, 170, 1);
        waitForMovement(robot, this, 10);

        turnDegrees(robot, this, 20, .2);
        waitForMovement(robot, this, 10);

        turnDegrees(robot,this, -20, .2);
        waitForMovement(robot, this, 10);
    }
}
