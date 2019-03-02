package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by jack on 10/27/18.
 */
@Autonomous(name = "crater auto no depot", group = "Linear Opmode")
public class CraterAutoNoDepot extends AutonomousBasic {

    @Override
    void runAutonomous() {
        double movementNeeded = disengageAndSample(robot, this);
        robot.drivetrain.strafeDistance(-16 + movementNeeded, .75);
        waitForMovement(robot, this, 1);
        turnDegrees(robot,this, -  35, .6);
        waitForMovement(robot, this, 1);
        robot.drivetrain.strafeDistance(-30, 1);
        waitForMovement(robot, this, .75);
        robot.drivetrain.strafeDistance(4, 1);
        waitForMovement(robot, this, .75);

        robot.drivetrain.driveDistance(24, 1);
        waitForMovement(robot, this, 10);

        robot.intake.extend();
        sleep(250);

        //for simple crater
//
//        robot.drivetrain.driveDistance(28,1);
//        waitForMovement(robot.drivetrain, this, 1);
    }
}
