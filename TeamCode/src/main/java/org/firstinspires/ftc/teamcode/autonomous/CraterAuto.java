package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by jack on 10/27/18.
 */
@Autonomous(name = "crater auto", group = "Linear Opmode")
public class CraterAuto extends AutonomousBasic {

    @Override
    void runAutonomous() {
        double movementNeeded = disengageAndSample(robot, this);
//        robot.drivetrain.strafeDistance(-16 + movementNeeded, .75);
//        waitForMovement(robot, this, 3);
//        turnDegrees(robot,this, 35, .2);
//        waitForMovement(robot, this, 3);
//        robot.drivetrain.strafeDistance(-10, .6);
//        waitForMovement(robot, this, .75);
//        robot.drivetrain.strafeDistance(4, .75);
//        waitForMovement(robot, this, 3);
//        robot.drivetrain.driveDistance(-48, 1);
//        //        waitForMovement(robot, this, 3);
////
////        turnDegrees(robot, this, -45, .6);
////        waitForMovement(robot, this, 3);
////
////        robot.intake.scoreMarker();
////        sleep(500);
////        robot.intake.openGate();
////
////        turnDegrees(robot, this, 45, .6);
////        waitForMovement(robot, this, 3);
////
////        robot.drivetrain.driveDistance(66, 1);
//
//        robot.drivetrain.driveDistance(18);

        robot.drivetrain.driveDistance(28,1);
        waitForMovement(robot, this, 1);
    }
}
